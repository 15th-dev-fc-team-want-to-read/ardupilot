-- command a Copter to takeoff and snap roll fly
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 'takeoff_alt_above_home' meters
--    c) rotate to 'yaw_target'
--    d) cruise to x direction, and ascent or descent until the altitude becomes 2 * 'takeoff_alt_above_home' meters
--    e) sequential snap roll until the altitude hits 'takeoff_alt_above_home' or 3 * 'takeoff_alt_above_home' meters
--    h) goto (d)

local takeoff_alt_above_home = 30.0  -- m
local copter_guided_mode_num = 4
local stage = 0
local cruise_pitch_angle = -30.0  -- deg
local cruise_climbrate  -- m/s, will be defined in several stages
local yaw_target = 0.0

-- the main update function that uses the takeoff and snap roll
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          stage = 1
          gcs:send_text(0, "transition to stage1: Takeoff")
        end

      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = 2
          gcs:send_text(0, "transition to stage2: Alt hold")
      end

      elseif (stage == 2) then      -- Stage2: altitude hold
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
            stage = 3
            gcs:send_text(0, "transition to stage3: Cruise to X")
            cruise_climbrate = 3.0  -- m/sec
          end
        end

      elseif (stage == 3) then   -- Stage3: cruise to X
        -- -- send angle request
        if not (vehicle:set_target_angle_and_climbrate(0.0, cruise_pitch_angle, yaw_target, cruise_climbrate, false, 0.0)) then
          gcs:send_text(0, "failed to execute angle command")
        end

        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          if (cruise_climbrate > 0.0) and (vec_from_home:z() < - 2 * takeoff_alt_above_home) then  -- ascent
            stage = 4
            gcs:send_text(0, "transition to stage4: Sequencial snap roll")
          elseif (cruise_climbrate < 0.0) and (vec_from_home:z() > - 2 * takeoff_alt_above_home) then  -- descent
            stage = 4
            gcs:send_text(0, "transition to stage4: Sequencial snap roll")
          end
        end

      elseif (stage == 4) then  -- Stage4: sequential snap roll
        -- rate axis is not only roll axis, but a combination of roll and yaw axis
        local roll_rate_rs = math.rad(180.0)
        local pitch_rate_rs = 0.0
        local yaw_rate_rs = -roll_rate_rs * math.cos(math.rad(30.0))

        -- set zero thrust during inverted flight
        local thrust = 1.0
        local roll_deg = math.deg(ahrs:get_roll())
        if (roll_deg < -90  or  90 < roll_deg) then
          thrust = 0.0
        end

        -- send rate request
        if not (vehicle:set_target_rate_and_thrust(roll_rate_rs, pitch_rate_rs, yaw_rate_rs, thrust)) then
          gcs:send_text(0, "failed to execute rate command")
        end

        -- transition to cruise when reaching the threshold
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          if (vec_from_home:z() > -takeoff_alt_above_home) then
            stage = 3
            gcs:send_text(0, "transition to stage4: Cruise to X")
            cruise_climbrate = 3.0  -- m/sec
          elseif (vec_from_home:z() < -3 * takeoff_alt_above_home) then
            stage = 3
            gcs:send_text(0, "transition to stage4: Cruise to X")
            cruise_climbrate = -3.0  -- m/sec
          end
        end
      end

    end
  end

  return update, 100
end

return update()
