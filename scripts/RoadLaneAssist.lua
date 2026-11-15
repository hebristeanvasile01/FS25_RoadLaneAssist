-- Road Lane Assist (FS25) - keeps the controlled vehicle in its lane on public roads.
-- Road data: the map's AI road splines (g_currentMission.aiSystem.roadSplines).
-- Steering: injected through Drivable's steering input (spec.lastInputValues.axisSteer).

RoadLaneAssist = {}
local RLA = RoadLaneAssist

RLA.state = "off"            -- off | calibrating | active
RLA.vehicle = nil
RLA.cache = nil
RLA.cacheCount = -1
RLA.axisOut = 0
RLA.keys = {}
RLA.modDirectory = g_currentModDirectory
RLA.settings = { offset = 0.0, sens = 1.0, showPath = false }
RLA.SETTINGS_FILE = "FS25_RoadLaneAssist.xml"

local JUNCTION_RADIUS = 5.0  -- another road this close to our path counts as a junction
local SAMPLE_STEP = 3.0      -- meters between cached spline points
local MAX_ROAD_DIST = 8.0    -- max distance to a road for the assist to work
local MAX_LANE_ERROR = 4.5   -- disengage if farther than this from the lane line
local MIN_ENGAGE_KMH = 5
local WHEEL_BASE = 3.2       -- rough defaults, the loop is closed so exact values are not needed
local MAX_STEER = 0.55
local CAL_MIN_TURN = 0.02       -- minimum heading difference (radians) that counts as a clear result
local CAL_PHASE_MS = 900
local OVERRIDE_AXIS = 0.3
local OVERRIDE_MS = 120
local NO_ROAD_MS = 4000      -- keeps going straight through junctions for this long
local MIN_ALIGN = 0.55       -- ignore road pieces crossing the driving direction (cos of angle)

local function clamp(v, lo, hi)
    return math.max(lo, math.min(hi, v))
end

local function getVehicle()
    if g_localPlayer ~= nil and g_localPlayer.getCurrentVehicle ~= nil then
        return g_localPlayer:getCurrentVehicle()
    end
    return nil
end

local function altDown()
    return (Input.KEY_lalt ~= nil and Input.isKeyPressed(Input.KEY_lalt))
end

function RLA:settingsPath()
    if getUserProfileAppPath == nil then
        return nil
    end
    return getUserProfileAppPath() .. "modSettings/" .. RLA.SETTINGS_FILE
end

function RLA:loadSettings()
    local path = self:settingsPath()
    if path == nil or fileExists == nil or loadXMLFile == nil or not fileExists(path) then
        return
    end
    local xml = loadXMLFile("RoadLaneAssistSettings", path)
    if xml == nil or xml == 0 then
        return
    end
    self.settings.offset = clamp(getXMLFloat(xml, "roadLaneAssist#laneOffset") or 0.0, 0.0, 4.5)
    self.settings.sens = clamp(getXMLFloat(xml, "roadLaneAssist#sensitivity") or 1.0, 0.6, 1.6)
    self.settings.showPath = (getXMLBool(xml, "roadLaneAssist#showPath") == true)
    delete(xml)
end

function RLA:saveSettings()
    local path = self:settingsPath()
    if path == nil or createFolder == nil or createXMLFile == nil then
        return
    end
    createFolder(getUserProfileAppPath() .. "modSettings/")
    local xml = createXMLFile("RoadLaneAssistSettings", path, "roadLaneAssist")
    if xml == nil or xml == 0 then
        return
    end
    setXMLFloat(xml, "roadLaneAssist#laneOffset", self.settings.offset)
    setXMLFloat(xml, "roadLaneAssist#sensitivity", self.settings.sens)
    setXMLBool(xml, "roadLaneAssist#showPath", self.settings.showPath)
    saveXMLFile(xml)
    delete(xml)
end

function RLA:notify(text, isWarning)
    print("[RoadLaneAssist] " .. text)
    if g_currentMission == nil then
        return
    end
    if isWarning or FSBaseMission.INGAME_NOTIFICATION_OK == nil then
        g_currentMission:showBlinkingWarning(text, 2500)
    else
        g_currentMission:addIngameNotification(FSBaseMission.INGAME_NOTIFICATION_OK, text)
    end
end

----------------------------------------------------------------------------
-- road data
----------------------------------------------------------------------------
function RLA:buildCache()
    self.cache = {}
    local aiSystem = g_currentMission ~= nil and g_currentMission.aiSystem or nil
    local splines = aiSystem ~= nil and aiSystem.roadSplines or nil
    self.cacheCount = splines ~= nil and #splines or 0
    if splines == nil then
        return
    end

    for _, spline in ipairs(splines) do
        local length = getSplineLength(spline)
        if length > 1 then
            local n = math.max(2, math.ceil(length / SAMPLE_STEP))
            local pts = {}
            local minX, maxX, minZ, maxZ = math.huge, -math.huge, math.huge, -math.huge
            for i = 0, n do
                local x, _, z = getSplinePosition(spline, i / n)
                pts[i + 1] = { x, z }
                minX = math.min(minX, x)
                maxX = math.max(maxX, x)
                minZ = math.min(minZ, z)
                maxZ = math.max(maxZ, z)
            end
            table.insert(self.cache, { pts = pts, minX = minX, maxX = maxX, minZ = minZ, maxZ = maxZ })
        end
    end
    print(string.format("[RoadLaneAssist] cached %d road splines", #self.cache))
end

function RLA:ensureCache()
    local aiSystem = g_currentMission ~= nil and g_currentMission.aiSystem or nil
    local count = (aiSystem ~= nil and aiSystem.roadSplines ~= nil) and #aiSystem.roadSplines or 0
    if self.cache == nil or count ~= self.cacheCount then
        self:buildCache()
    end
end

---Nearest point on any cached road spline. Returns nil if none within MAX_ROAD_DIST.
function RLA:findRoad(px, pz, fx, fz, minAlign)
    minAlign = minAlign or MIN_ALIGN
    local best = nil
    local bestScore = nil
    local r = MAX_ROAD_DIST
    for idx, sp in ipairs(self.cache) do
        if px > sp.minX - r and px < sp.maxX + r and pz > sp.minZ - r and pz < sp.maxZ + r then
            local pts = sp.pts
            local bias = (idx == self.curSpline) and 0.4 or 1.0
            for i = 1, #pts - 1 do
                local ax, az = pts[i][1], pts[i][2]
                local abx, abz = pts[i + 1][1] - ax, pts[i + 1][2] - az
                local len2 = abx * abx + abz * abz
                local aligned = true
                if fx ~= nil and len2 > 1e-6 then
                    local dot = (abx * fx + abz * fz) / math.sqrt(len2)
                    aligned = math.abs(dot) >= minAlign
                end
                if len2 > 1e-6 and aligned then
                    local t = clamp(((px - ax) * abx + (pz - az) * abz) / len2, 0, 1)
                    local dx, dz = px - (ax + abx * t), pz - (az + abz * t)
                    local d2 = dx * dx + dz * dz
                    if d2 <= r * r and (bestScore == nil or d2 * bias < bestScore) then
                        bestScore = d2 * bias
                        best = { spline = idx, seg = i, t = t, d2 = d2, dx = dx, dz = dz }
                    end
                end
            end
        end
    end
    return best
end

---Nearest point on the spline we are already following, searched only near the last position.
-- Keeps the tracking from jumping to another part of the road or to a neighbouring road.
function RLA:findLocal(px, pz)
    local sp = self.curSpline ~= nil and self.cache[self.curSpline] or nil
    if sp == nil or self.curSeg == nil then
        return nil
    end
    local pts = sp.pts
    local lo = math.max(1, self.curSeg - 8)
    local hi = math.min(#pts - 1, self.curSeg + 8)
    local best = nil
    for i = lo, hi do
        local ax, az = pts[i][1], pts[i][2]
        local abx, abz = pts[i + 1][1] - ax, pts[i + 1][2] - az
        local len2 = abx * abx + abz * abz
        if len2 > 1e-6 then
            local t = clamp(((px - ax) * abx + (pz - az) * abz) / len2, 0, 1)
            local dx, dz = px - (ax + abx * t), pz - (az + abz * t)
            local d2 = dx * dx + dz * dz
            if best == nil or d2 < best.d2 then
                best = { spline = self.curSpline, seg = i, t = t, d2 = d2, dx = dx, dz = dz }
            end
        end
    end
    return best
end

local function segDir(pts, i)
    i = clamp(i, 1, #pts - 1)
    local abx, abz = pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]
    local len = math.sqrt(abx * abx + abz * abz)
    if len < 1e-6 then
        return 0, 1
    end
    return abx / len, abz / len
end

---Point `dist` meters ahead of the road position, travelling in direction `sgn` (+1 / -1 along the spline).
-- Returns x, z and the travel direction there. Past the spline end the last direction is extended.
local function walkAhead(pts, seg, t, sgn, dist)
    local i = seg
    local frac = t
    local remaining = dist
    local dx, dz = segDir(pts, i)
    local x = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * frac
    local z = pts[i][2] + (pts[i + 1][2] - pts[i][2]) * frac
    while remaining > 0 do
        dx, dz = segDir(pts, i)
        local ax, az, bx, bz
        if sgn > 0 then
            ax, az, bx, bz = x, z, pts[i + 1][1], pts[i + 1][2]
        else
            ax, az, bx, bz = x, z, pts[i][1], pts[i][2]
        end
        local segLeft = math.sqrt((bx - ax) ^ 2 + (bz - az) ^ 2)
        if segLeft >= remaining then
            local k = remaining / math.max(segLeft, 1e-6)
            return ax + (bx - ax) * k, az + (bz - az) * k, dx * sgn, dz * sgn
        end
        remaining = remaining - segLeft
        x, z = bx, bz
        i = i + sgn
        if i < 1 or i > #pts - 1 then
            return x + dx * sgn * remaining, z + dz * sgn * remaining, dx * sgn, dz * sgn
        end
    end
    return x, z, dx * sgn, dz * sgn
end

----------------------------------------------------------------------------
-- engage / disengage
----------------------------------------------------------------------------
function RLA:disengage(reason, warn)
    if self.state ~= "off" then
        self:notify("Lane Assist off: " .. tostring(reason), warn ~= false)
    end
    self.state = "off"
    self.vehicle = nil
    self.lastWritten = 0
    self.branch = nil
    self.force = nil
    self.axisOut = 0
    self.cmdSmooth = 0
    self.curSpline = nil
end

local function isArticulated(vehicle)
    local spec = vehicle.spec_articulatedAxis
    return spec ~= nil and spec.componentJoint ~= nil and vehicle.getSteeringRotTimeByCurvature ~= nil
end

local function vehicleFrame(vehicle)
    local node = vehicle.steeringAxleNode or vehicle.rootNode
    local px, _, pz = getWorldTranslation(node)
    local fx, _, fz = localDirectionToWorld(node, 0, 0, 1)
    local xx, _, xz = localDirectionToWorld(node, 1, 0, 0)
    local fl = math.max(math.sqrt(fx * fx + fz * fz), 1e-6)
    local xl = math.max(math.sqrt(xx * xx + xz * xz), 1e-6)
    return px, pz, fx / fl, fz / fl, xx / xl, xz / xl
end

function RLA:engage()
    local vehicle = getVehicle()
    if vehicle == nil or vehicle.spec_drivable == nil then
        self:notify("Lane Assist: get into a vehicle first", true)
        return
    end
    if vehicle:getLastSpeed() < MIN_ENGAGE_KMH then
        self:notify("Lane Assist: drive on the road first", true)
        return
    end

    self:ensureCache()
    if #self.cache == 0 then
        self:notify("Lane Assist: this map has no road data", true)
        return
    end

    local px, pz, fx, fz, xx, xz = vehicleFrame(vehicle)
    local road = self:findRoad(px, pz, fx, fz)
    if road == nil then
        self:notify("Lane Assist: no road nearby", true)
        return
    end

    self:dumpNear(px, pz, fx, fz, xx, xz, "engage")
    local pts = self.cache[road.spline].pts
    local dx, dz = segDir(pts, road.seg)
    local sgn = (dx * fx + dz * fz) >= 0 and 1 or -1
    local tx, tz = dx * sgn, dz * sgn
    -- which perpendicular of the travel direction is the vehicle's +x side
    local qx, qz = tz, -tx
    local hand = (qx * xx + qz * xz) >= 0 and 1 or -1
    local lat = hand * (road.dx * qx + road.dz * qz)   -- lateral position, positive on the +x side

    -- default: drive on the road centre line; Alt + Up/Down moves the lane away from it
    local leftHand = g_currentMission.aiSystem ~= nil and g_currentMission.aiSystem.isLeftHandTraffic
    self.laneSide = lat ~= 0 and (lat >= 0 and 1 or -1) or (leftHand and 1 or -1)
    if math.abs(lat) < 0.8 then
        self.laneSide = leftHand and 1 or -1
    end
    self.laneOffset = self.settings.offset

    self.vehicle = vehicle
    self.curSpline = road.spline
    self.curSeg = road.seg
    self.curSgn = sgn
    self.revMs = 0
    self.cmdSmooth = 0
    self.noRoadMs = 0
    self.overrideMs = 0
    self.road = nil
    self.path = nil
    self.laneError = 0
    self.branch = nil
    self.force = nil

    -- game convention (Drivable): positive steer input = right, scaled by the steering direction.
    -- The measured calibration proved unreliable in curves, so it is not used.
    vehicle.rlaAxisSign = -((vehicle.getSteeringDirection ~= nil and vehicle:getSteeringDirection()) or 1)
    if true then
        self.state = "active"
        self:notify("Lane Assist on", false)
    else
        self.state = "calibrating"
        self.calMs = 0
        self.calPhase = 1
        self.calStartF = { fx, fz }
        self.calStartX = { xx, xz }
        self.calDelta = {}
        -- small test input: the heading change it causes grows with speed, so scale it down
        self.calAmount = clamp(0.5 / math.max(vehicle:getLastSpeed() / 3.6, 3), 0.03, 0.12)
        self.calMid = nil
        if isArticulated(vehicle) then
            -- articulated steering is slow: longer phases, measure only the second half of each
            self.calAmount = clamp(self.calAmount, 0.03, 0.12)
        end
        self:notify("Lane Assist: calibrating steering - hands off", false)
    end
end

function RLA:flipSide()
    if self.state == "off" then
        return
    end
    self.laneSide = -self.laneSide
    self:notify("Lane Assist: lane side switched", false)
end

----------------------------------------------------------------------------
-- control
----------------------------------------------------------------------------
function RLA:updateCalibration(vehicle, dt)
    self.calMs = self.calMs + dt
    local _, _, fx, fz = vehicleFrame(vehicle)

    if vehicle:getLastSpeed() < 3 then
        self:disengage("too slow to calibrate", true)
        return
    end

    self.axisOut = (self.calPhase == 1) and self.calAmount or -self.calAmount

    local artic = isArticulated(vehicle)
    local phaseMs = artic and (CAL_PHASE_MS * 2.2) or CAL_PHASE_MS
    if artic and self.calMid == nil and self.calMs >= phaseMs * 0.5 then
        local _, _, mfx, mfz, mxx, mxz = vehicleFrame(vehicle)
        self.calMid = { mfx, mfz, mxx, mxz }
    end

    if self.calMs >= phaseMs then
        -- turn toward the start +x side, measured against the heading at the start of this phase
        local refX = self.calStartX
        if self.calMid ~= nil then
            refX = { self.calMid[3], self.calMid[4] }
        end
        self.calDelta[self.calPhase] = fx * refX[1] + fz * refX[2]
        self.calMid = nil
        if self.calPhase == 1 then
            self.calPhase = 2
            self.calMs = 0
            local _, _, _, _, xx, xz = vehicleFrame(vehicle)
            self.calStartF = { fx, fz }
            self.calStartX = { xx, xz }
        else
            local diff = self.calDelta[1] - self.calDelta[2]
            self.axisOut = 0
            if math.abs(diff) < CAL_MIN_TURN then
                self:disengage("calibration failed, try again at a higher speed", true)
                return
            end
            vehicle.rlaAxisSign = diff > 0 and 1 or -1
            print(string.format("[RoadLaneAssist] calibrated: sign=%d diff=%.3f amount=%.3f d1=%.3f d2=%.3f", vehicle.rlaAxisSign, diff, self.calAmount, self.calDelta[1], self.calDelta[2]))
            self.state = "active"
            self:notify("Lane Assist on", false)
        end
    end
end

----------------------------------------------------------------------------
-- turn signal: take a turn only at a junction / roundabout exit
----------------------------------------------------------------------------
---1 = left signal, -1 = right signal, 0 = none (hazard lights are ignored)
function RLA:getSignal(vehicle)
    local spec = vehicle.spec_lights
    if spec == nil or Lights == nil then
        return 0
    end
    if spec.turnLightState == Lights.TURNLIGHT_LEFT then
        return 1
    end
    if spec.turnLightState == Lights.TURNLIGHT_RIGHT then
        return -1
    end
    return 0
end

---Looks ahead on our road for another road that leaves it to the signalled side.
-- Returns nil when there is no junction ahead, so the signal alone does nothing.
function RLA:dumpNear(px, pz, fx, fz, xx, xz, tag)
    local out = {}
    for idx, sp in ipairs(self.cache) do
        if px > sp.minX - 12 and px < sp.maxX + 12 and pz > sp.minZ - 12 and pz < sp.maxZ + 12 then
            local bd, info = nil, nil
            local pts = sp.pts
            for i = 1, #pts - 1 do
                local ax, az = pts[i][1], pts[i][2]
                local abx, abz = pts[i + 1][1] - ax, pts[i + 1][2] - az
                local len2 = abx * abx + abz * abz
                if len2 > 1e-6 then
                    local t = clamp(((px - ax) * abx + (pz - az) * abz) / len2, 0, 1)
                    local ex, ez = ax + abx * t - px, az + abz * t - pz
                    local d = math.sqrt(ex * ex + ez * ez)
                    if bd == nil or d < bd then
                        local len = math.sqrt(len2)
                        bd = d
                        info = string.format("%d(d=%.1f dir=%+.2f left=%+.1f n=%d)", idx, d,
                            (abx * fx + abz * fz) / len, ex * xx + ez * xz, #pts)
                    end
                end
            end
            if bd ~= nil and bd < 12 then
                out[#out + 1] = { bd, info }
            end
        end
    end
    table.sort(out, function(a, b) return a[1] < b[1] end)
    local strs = {}
    for k = 1, math.min(#out, 8) do
        strs[#strs + 1] = out[k][2]
    end
    print("[RoadLaneAssist] near " .. tag .. ": " .. table.concat(strs, " "))
end

function RLA:findBranch(xx, xz, fx, fz, pts, seg, t, sgn, signal, speed)
    local maxD = clamp(speed * 3 + 12, 24, 70)
    local r2 = JUNCTION_RADIUS * JUNCTION_RADIUS
    local best = nil
    local cands = {}
    local d = 3
    while d <= maxD do
        local sx, sz = walkAhead(pts, seg, t, sgn, d)
        for idx, sp in ipairs(self.cache) do
            if idx ~= self.curSpline and sx > sp.minX - 5 and sx < sp.maxX + 5 and sz > sp.minZ - 5 and sz < sp.maxZ + 5 then
                local opts = sp.pts
                for i = 1, #opts - 1 do
                    local ax, az = opts[i][1], opts[i][2]
                    local abx, abz = opts[i + 1][1] - ax, opts[i + 1][2] - az
                    local len2 = abx * abx + abz * abz
                    if len2 > 1e-6 then
                        local tt = clamp(((sx - ax) * abx + (sz - az) * abz) / len2, 0, 1)
                        local ex, ez = sx - (ax + abx * tt), sz - (az + abz * tt)
                        if ex * ex + ez * ez < r2 then
                            local len = math.sqrt(len2)
                            for _, o in ipairs({ 1, -1 }) do
                                local cx, cz = abx / len * o, abz / len * o
                                -- the road must continue away from the junction in this direction
                                local segsLeft = (o > 0) and (#opts - i - tt) or (i - 1 + tt)
                                if segsLeft * SAMPLE_STEP >= 9 then
                                    local ang = math.atan2(cx * xx + cz * xz, cx * fx + cz * fz) -- + = left
                                    local want = signal * ang
                                    if want > 0.35 and want < 2.6 then
                                        local score = d + 4 * want
                                        local cand = { score = score, spline = idx, seg = i, t = tt, sgn = o,
                                            jx = ax + abx * tt, jz = az + abz * tt, signal = signal, cx = cx, cz = cz, dist = d }
                                        cands[#cands + 1] = cand
                                        if best == nil or score < best.score then
                                            best = cand
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        d = d + SAMPLE_STEP
    end

    if best ~= nil and best.dist + 9 > maxD then
        return nil   -- wait until the neighbouring lane lines are in view as well
    end
    if best ~= nil then
        -- roads can have one line per lane: take the line of the lane that belongs to the new
        -- direction of travel (right-hand traffic: the right one, left-hand traffic: the left one)
        local aiSystem = g_currentMission ~= nil and g_currentMission.aiSystem or nil
        local leftHand = aiSystem ~= nil and aiSystem.isLeftHandTraffic == true
        local hand = (fz * xx - fx * xz) >= 0 and 1 or -1
        local rx, rz = -hand * best.cz, hand * best.cx
        local pick, pickLat = best, best.jx * rx + best.jz * rz
        for _, c in ipairs(cands) do
            local near = (c.jx - best.jx) ^ 2 + (c.jz - best.jz) ^ 2 < 196
            if near and c.cx * best.cx + c.cz * best.cz > 0.94 then
                local lat = c.jx * rx + c.jz * rz
                if (not leftHand and lat > pickLat) or (leftHand and lat < pickLat) then
                    pick, pickLat = c, lat
                end
            end
        end
        print(string.format("[RoadLaneAssist] branch: %d candidates, best spline %d, chosen spline %d (leftHandTraffic=%s)",
            #cands, best.spline, pick.spline, tostring(leftHand)))
        best = pick
    end
    return best
end

---Nearest point on the road we are turning onto (any heading).
function RLA:findForced(px, pz)
    local sp = self.cache[self.force.spline]
    if sp == nil then
        return nil
    end
    local pts = sp.pts
    local best = nil
    for i = 1, #pts - 1 do
        local ax, az = pts[i][1], pts[i][2]
        local abx, abz = pts[i + 1][1] - ax, pts[i + 1][2] - az
        local len2 = abx * abx + abz * abz
        if len2 > 1e-6 then
            local t = clamp(((px - ax) * abx + (pz - az) * abz) / len2, 0, 1)
            local dx, dz = px - (ax + abx * t), pz - (az + abz * t)
            local d2 = dx * dx + dz * dz
            if best == nil or d2 < best.d2 then
                best = { spline = self.force.spline, seg = i, t = t, d2 = d2, dx = dx, dz = dz }
            end
        end
    end
    return best
end

function RLA:cancelSignal(vehicle)
    if vehicle.setTurnLightState ~= nil and Lights ~= nil then
        vehicle:setTurnLightState(Lights.TURNLIGHT_OFF)
    end
end

function RLA:updateActive(vehicle, dt)
    if vehicle.movingDirection ~= nil and vehicle.movingDirection < 0 then
        self.revMs = (self.revMs or 0) + dt
        if self.revMs > 500 then
            self:disengage("reversing", true)
            return
        end
    else
        self.revMs = 0
    end

    local px, pz, fx, fz, xx, xz = vehicleFrame(vehicle)

    -- road search: stay on the spline we follow; only look for another one at its end or when lost
    local reacquired = false
    if self.force ~= nil then
        self.road = self:findForced(px, pz)
    else
        local loc = self:findLocal(px, pz)
        local locOk = loc ~= nil and loc.d2 <= MAX_ROAD_DIST * MAX_ROAD_DIST
        local atEnd = false
        if locOk then
            local n = #self.cache[loc.spline].pts
            atEnd = (self.curSgn > 0 and loc.seg >= n - 2) or (self.curSgn < 0 and loc.seg <= 2)
        end
        if locOk and not atEnd then
            self.road = loc
        else
            -- continuing onto the next road: only roads that go straight on (no turning without the signal)
            local glob = self:findRoad(px, pz, fx, fz, 0.9)
            if glob == nil and not locOk then
                glob = self:findRoad(px, pz, fx, fz)
            end
            if glob ~= nil and (not locOk or glob.spline ~= loc.spline) then
                self.road = glob
                reacquired = true
            else
                self.road = locOk and loc or glob
                reacquired = not locOk and glob ~= nil
            end
        end
    end
    if self.road ~= nil and self.force == nil then
        self.curSpline = self.road.spline
        self.curSeg = self.road.seg
    end

    if self.road == nil then
        self.noRoadMs = self.noRoadMs + dt
        self.axisOut = 0
        self.cmdSmooth = 0
        self.path = nil
        if self.noRoadMs > NO_ROAD_MS then
            self:disengage("left the road", true)
        end
        return
    end
    self.noRoadMs = 0

    local pts = self.cache[self.road.spline].pts
    local dx, dz = segDir(pts, self.road.seg)
    if reacquired then
        self:dumpNear(px, pz, fx, fz, xx, xz, "switch")
        self.curSgn = (dx * fx + dz * fz) >= 0 and 1 or -1
    end
    local sgn = self.curSgn
    if self.force ~= nil then
        sgn = self.force.sgn
    end
    local tx, tz = dx * sgn, dz * sgn
    local qx, qz = tz, -tx
    local hand = (fz * xx - fx * xz) >= 0 and 1 or -1
    local lat = hand * (self.road.dx * qx + self.road.dz * qz)
    local laneError = lat - self.laneSide * self.laneOffset
    self.laneError = laneError

    local speed = vehicle:getLastSpeed() / 3.6
    local signal = self:getSignal(vehicle)

    -- junction handling: the signal only does something where a road really branches off
    local jdist = nil
    if self.force ~= nil then
        self.force.ms = self.force.ms + dt
        local aligned = tx * fx + tz * fz
        if (aligned > 0.93 and self.force.ms > 1200) or self.force.ms > 12000 then
            self:dumpNear(px, pz, fx, fz, xx, xz, "turn end")
            self.curSgn = self.force.sgn
            self.curSeg = self.road.seg
            self.force = nil
            self.curSpline = self.road.spline
            self:cancelSignal(vehicle)
        end
    elseif signal ~= 0 then
        self.branchMs = (self.branchMs or 0) + dt
        if self.branch ~= nil and self.branch.signal ~= signal then
            self.branch = nil
        end
        if self.branch == nil and self.branchMs >= 150 then
            self.branchMs = 0
            self.branch = self:findBranch(xx, xz, fx, fz, pts, self.road.seg, self.road.t, sgn, signal, speed)
        end
        if self.branch ~= nil then
            jdist = (self.branch.jx - px) * fx + (self.branch.jz - pz) * fz
            if jdist < -8 then
                self.branch = nil
                jdist = nil
            elseif jdist < math.max(3, speed * 0.3) then
                self.force = { spline = self.branch.spline, sgn = self.branch.sgn, ms = 0 }
                self.curSpline = self.branch.spline
                self:notify(signal > 0 and "Lane Assist: turning left" or "Lane Assist: turning right", false)
                self.branch = nil
                jdist = nil
            end
        end
    else
        self.branch = nil
    end

    local tooFar = (self.force ~= nil) and 25 or (MAX_LANE_ERROR + 2.5)
    if math.abs(laneError) > tooFar then
        self:disengage("too far from the lane", true)
        return
    end

    local artic = isArticulated(vehicle)
    local lookahead = clamp((speed * 1.1 + 4) / self.settings.sens, 6, 40)
    if artic then
        lookahead = clamp(lookahead * 1.5, 10, 50)
    end
    local turning = self.force ~= nil or (self.branch ~= nil and jdist ~= nil and jdist < lookahead + 6)
    if turning then
        lookahead = clamp(lookahead * 0.6, 6, 16)
    elseif self.force == nil then
        -- in a curve a long lookahead cuts the corner: shorten it by the curve's radius
        local _, _, d1x, d1z = walkAhead(pts, self.road.seg, self.road.t, sgn, lookahead * 0.6)
        local turn = math.acos(clamp(tx * d1x + tz * d1z, -1, 1))
        local kappa = math.max(turn / (lookahead * 0.6), 0.002)
        local minL = artic and 8 or math.max(5, speed * 0.5)
        lookahead = clamp(math.min(lookahead, math.sqrt(2.4 / kappa)), math.min(minL, lookahead), lookahead)
    end
    local ax, az, adx, adz = walkAhead(pts, self.road.seg, self.road.t, sgn, lookahead)
    if self.force == nil and self.branch ~= nil and jdist ~= nil and jdist < lookahead then
        -- aim along the road we are about to turn onto
        local bpts = self.cache[self.branch.spline].pts
        ax, az, adx, adz = walkAhead(bpts, self.branch.seg, self.branch.t, self.branch.sgn, math.max(lookahead - jdist, 4))
    end
    local px2, pz2 = adz, -adx
    local ox, oz = hand * px2, hand * pz2
    local targetX = ax + ox * self.laneSide * self.laneOffset
    local targetZ = az + oz * self.laneSide * self.laneOffset

    if self.settings.showPath then
        local path = {}
        for k = 0, 8 do
            local qx2, qz2, qdx, qdz = walkAhead(pts, self.road.seg, self.road.t, sgn, 4 + k * 5)
            local ox2, oz2 = hand * qdz, hand * -qdx
            path[#path + 1] = { qx2 + ox2 * self.laneSide * self.laneOffset, qz2 + oz2 * self.laneSide * self.laneOffset }
        end
        self.path = path
    else
        self.path = nil
    end

    local rx, rz = targetX - px, targetZ - pz
    local lx = rx * xx + rz * xz     -- target offset toward the vehicle's +x side
    local lz = rx * fx + rz * fz     -- target distance ahead
    local d2 = math.max(lx * lx + lz * lz, 1)
    local curvature = 2 * lx / d2
    local cmd = clamp(math.atan(curvature * WHEEL_BASE) / MAX_STEER, -1, 1)
    local smoothMs = 150
    if artic then
        -- use the vehicle's own steering geometry: how much articulation this curvature needs
        local rotTime = vehicle:getSteeringRotTimeByCurvature(math.abs(curvature))
        local maxRot = math.max(math.abs(vehicle.maxRotTime or 0), math.abs(vehicle.minRotTime or 0))
        if rotTime ~= nil and maxRot > 0 then
            cmd = clamp(math.abs(rotTime) / maxRot, 0, 1) * (curvature >= 0 and 1 or -1)
        end
        smoothMs = 350
    end

    self.cmdSmooth = self.cmdSmooth + (cmd - self.cmdSmooth) * math.min(1, dt / smoothMs)
    vehicle.rlaAxisSign = -((vehicle.getSteeringDirection ~= nil and vehicle:getSteeringDirection()) or 1)
    self.axisOut = clamp(vehicle.rlaAxisSign * self.cmdSmooth, -1, 1)

    self.dbgMs = (self.dbgMs or 0) + dt
    if self.dbgMs >= 1000 then
        self.dbgMs = 0
        print(string.format("[RoadLaneAssist] dbg v=%.1f spline=%d seg=%d latErr=%.2f roadDist=%.2f look=%.1f lx=%.2f lz=%.2f cmd=%.2f sign=%d force=%s",
            speed * 3.6, self.road.spline, self.road.seg, self.laneError, math.sqrt(self.road.d2 or 0), lookahead, lx, lz,
            self.cmdSmooth, vehicle.rlaAxisSign or 0, tostring(self.force ~= nil)))
    end
end

---Edge detected Left Alt + key combinations.
function RLA:handleKeys()
    local alt = altDown()
    local function pressed(name, key)
        local down = alt and key ~= nil and Input.isKeyPressed(key)
        local hit = down and not self.keys[name]
        self.keys[name] = down
        return hit
    end

    -- toggle: Insert (single key), or Left Alt + W as a fallback
    local toggleDown = Input.KEY_insert ~= nil and Input.isKeyPressed(Input.KEY_insert)
    if not toggleDown and alt and Input.KEY_w ~= nil and Input.isKeyPressed(Input.KEY_w) then
        toggleDown = true
    end
    local toggleHit = toggleDown and not self.keys.toggle
    self.keys.toggle = toggleDown

    if toggleHit then
        if self.state == "off" then
            self:engage()
        else
            self:disengage("switched off", false)
        end
    end
    if pressed("side", Input.KEY_k) then
        self:flipSide()
    end
    if pressed("wider", Input.KEY_up) and self.state ~= "off" then
        self.laneOffset = clamp(self.laneOffset + 0.25, 0.0, 4.5)
        self.settings.offset = self.laneOffset
        self:saveSettings()
        self:notify(string.format("Lane Assist: lane %.2f m from road centre", self.laneOffset), false)
    end
    if pressed("closer", Input.KEY_down) and self.state ~= "off" then
        self.laneOffset = clamp(self.laneOffset - 0.25, 0.0, 4.5)
        self.settings.offset = self.laneOffset
        self:saveSettings()
        self:notify(string.format("Lane Assist: lane %.2f m from road centre", self.laneOffset), false)
    end
    if pressed("sensUp", Input.KEY_pageup) then
        self.settings.sens = clamp(self.settings.sens + 0.1, 0.6, 1.6)
        self:saveSettings()
        self:notify(string.format("Lane Assist: response %.1f", self.settings.sens), false)
    end
    if pressed("sensDown", Input.KEY_pagedown) then
        self.settings.sens = clamp(self.settings.sens - 0.1, 0.6, 1.6)
        self:saveSettings()
        self:notify(string.format("Lane Assist: response %.1f", self.settings.sens), false)
    end
    if pressed("path", Input.KEY_h) then
        self.settings.showPath = not self.settings.showPath
        self:saveSettings()
        self:notify("Lane Assist: path display " .. (self.settings.showPath and "on" or "off"), false)
    end
end

function RLA:update(dt)
    if g_dedicatedServer ~= nil or g_currentMission == nil or g_gui == nil then
        return
    end

    self:handleKeys()

    if self.state == "off" then
        return
    end

    local vehicle = getVehicle()
    if vehicle == nil or vehicle ~= self.vehicle then
        self:disengage("left the vehicle", false)
        return
    end

    if self.state == "calibrating" then
        self:updateCalibration(vehicle, dt)
    elseif self.state == "active" then
        self:updateActive(vehicle, dt)
    end
end

---Runs before Drivable:onUpdate each frame and supplies the steering input.
function RLA.onDrivableUpdate(vehicle, dt)
    if RLA.state == "off" or vehicle ~= RLA.vehicle then
        return
    end
    local spec = vehicle.spec_drivable
    if spec == nil or spec.lastInputValues == nil then
        return
    end

    -- the driver steers: hand control back
    if math.abs(spec.lastInputValues.axisSteer or 0) > OVERRIDE_AXIS then
        RLA.overrideMs = (RLA.overrideMs or 0) + dt
        if RLA.overrideMs > OVERRIDE_MS then
            RLA:disengage("manual steering", false)
            return
        end
    else
        RLA.overrideMs = 0
    end

    spec.lastInputValues.axisSteer = RLA.axisOut
end

function RLA:getWheelOverlay()
    if self.wheelOverlay == nil then
        local id = createImageOverlay(self.modDirectory .. "hud/steeringWheel.dds")
        self.wheelOverlay = (id ~= nil and id ~= 0) and id or false
    end
    return self.wheelOverlay or nil
end

function RLA:draw()
    if g_gui:getIsGuiVisible() then
        return
    end
    local vehicle = getVehicle()
    if vehicle == nil or vehicle.spec_drivable == nil then
        return
    end

    if self.state ~= "off" and self.settings.showPath and self.path ~= nil and drawDebugLine ~= nil and g_terrainNode ~= nil then
        for i = 1, #self.path - 1 do
            local a, b = self.path[i], self.path[i + 1]
            local ay = getTerrainHeightAtWorldPos(g_terrainNode, a[1], 0, a[2]) + 0.25
            local by = getTerrainHeightAtWorldPos(g_terrainNode, b[1], 0, b[2]) + 0.25
            drawDebugLine(a[1], ay, a[2], 0.55, 0.9, 0.3, b[1], by, b[2], 0.55, 0.9, 0.3)
        end
    end
end

-- Steering-wheel icon inside the vanilla speedometer (the guidance/GPS slot).
function RLA:isHudOwner(speedMeter)
    local vehicle = speedMeter.vehicle
    if vehicle == nil or vehicle ~= getVehicle() or vehicle.spec_drivable == nil then
        return false
    end
    if vehicle.getIsAIActive ~= nil and vehicle:getIsAIActive() then
        return false
    end
    return true
end

local function noRender()
end

function RLA.onSpeedMeterDrawBefore(speedMeter)
    local owner = RLA:isHudOwner(speedMeter)
    RLA.hudOwner = owner
    if owner then
        local text = "Off"
        if RLA.state == "active" then
            text = "On"
        elseif RLA.state == "calibrating" then
            text = "..."
        end
        RLA.hudOrigText = RLA.hudOrigText or {speedMeter.aiInactiveText, speedMeter.aiReadyText, speedMeter.aiActiveText}
        speedMeter.aiInactiveText, speedMeter.aiReadyText, speedMeter.aiActiveText = text, text, text
        speedMeter.aiWorkerIcon.render = noRender
        speedMeter.aiSteeringIcon.render = noRender
    else
        if RLA.hudOrigText ~= nil then
            speedMeter.aiInactiveText, speedMeter.aiReadyText, speedMeter.aiActiveText = unpack(RLA.hudOrigText)
            RLA.hudOrigText = nil
        end
        speedMeter.aiWorkerIcon.render = nil
        speedMeter.aiSteeringIcon.render = nil
    end
end

function RLA.onSpeedMeterDrawAfter(speedMeter)
    if not RLA.hudOwner or speedMeter.vehicle == nil or not speedMeter.isVehicleDrawSafe then
        return
    end
    local overlay = RLA:getWheelOverlay()
    if overlay == nil or speedMeter.aiIconOffsetX == nil then
        return
    end

    local posX, posY = speedMeter:getPosition()
    local w, h = speedMeter.aiSteeringIcon.width, speedMeter.aiSteeringIcon.height
    local x, y = posX + speedMeter.aiIconOffsetX, posY + speedMeter.aiIconOffsetY

    -- vehicles without the vanilla guidance row: draw separator + text ourselves
    if speedMeter.vehicle.getAIAutomaticSteeringState == nil then
        drawLine2D(posX + speedMeter.aiSeperatorOffsetX, posY + speedMeter.aiSeperatorOffsetY,
            posX + speedMeter.aiSeperatorOffsetX + speedMeter.aiSeperatorWidth, posY + speedMeter.aiSeperatorOffsetY,
            g_pixelSizeY, 0.55, 0.9, 0.3, 1)
    end

    -- orange = off (or no road), green = on, blinking green = calibrating
    local green = RLA.state == "active" and RLA.road ~= nil
    if RLA.state == "calibrating" then
        green = (g_time % 700) < 350
    end
    if green then
        setOverlayColor(overlay, 0.55, 0.9, 0.3, 1)
    else
        setOverlayColor(overlay, 1, 0.55, 0.1, 1)
    end
    renderOverlay(overlay, x, y, w, h)
end

if SpeedMeterDisplay ~= nil then
    SpeedMeterDisplay.draw = Utils.prependedFunction(SpeedMeterDisplay.draw, RLA.onSpeedMeterDrawBefore)
    SpeedMeterDisplay.draw = Utils.appendedFunction(SpeedMeterDisplay.draw, RLA.onSpeedMeterDrawAfter)
end

if Drivable ~= nil then
    Drivable.onUpdate = Utils.prependedFunction(Drivable.onUpdate, RLA.onDrivableUpdate)
end

RLA:loadSettings()

addModEventListener(RLA)
