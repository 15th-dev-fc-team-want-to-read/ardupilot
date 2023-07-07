-- command a Copter to takeoff and vertical loop fly
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 'takeoff_alt_above_home' meters
--    c) rotate to 'yaw_target'
--    d) cruise to x direction for 'cruise_counter_max1 / 10' sec
--    e) pitch up with 'pitch_up_rate1' deg/sec
--    f) cruise to -x direction for 'cruise_counter_max2 / 10' sec
--    g) pitch up with 'pitch_up_rate2' deg/sec
--    h) goto (d)

local takeoff_alt_above_home = 20  -- m
local copter_guided_mode_num = 4
local stage = 0
local cruise_counter = 0
local cruise_counter_max1 = 30  -- counts (10counts = 1sec)
local cruise_counter_max2 = 35  -- counts (10counts = 1sec)
local cruise_pitch_angle1 = -30.0  -- deg
local cruise_pitch_angle2 =  30.0  -- deg
local pitch_up_rate1 = 48.0  -- deg/sec
local pitch_up_angle_threshold1 = 20  -- deg
local pitch_up_rate2 = 160.0  -- deg/sec
local pitch_up_angle_threshold2 = -70  -- deg
local pitch_deg_prev = 1.0e8

local yaw_target = 0.0

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
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
            gcs:send_text(0, "transition to stage3: Yaw rotation")
          end
        end

      elseif (stage == 3) then  -- Stage3: yaw rotation
        cruise_counter = cruise_counter + 1

        if not (vehicle:set_target_angle_and_climbrate(0.0, 0.0, yaw_target, 0.0, false, 0.0)) then
          gcs:send_text(0, "failed to execute angle command")
        end

        if cruise_counter > 100 then
          cruise_counter = 0
          stage = 4
          gcs:send_text(0, "transition to stage4: Cruise to X")
        end

      elseif (stage == 4) then   -- Stage4: cruise to X
        cruise_counter = cruise_counter + 1

        if (cruise_counter > cruise_counter_max1) then
          stage = 5
          gcs:send_text(0, "transition to stage5: Pitch up")
        end

        -- -- send angle request
        if not (vehicle:set_target_angle_and_climbrate(0.0, cruise_pitch_angle1, yaw_target, 0.0, false, 0.0)) then
          gcs:send_text(0, "failed to execute angle command")
        end

      elseif (stage == 5) then  -- Stage5: pitch up and transition to cruise to -X
        local roll_rate_rs = 0.0
        local pitch_rate_rs = pitch_up_rate1 * math.rad(180.0) / 180.0
        local yaw_rate_rs = 0.0
        local thrust = 1.0

        -- send rate request
        if not (vehicle:set_target_rate_and_thrust(roll_rate_rs, pitch_rate_rs, yaw_rate_rs, thrust)) then
          gcs:send_text(0, "failed to execute rate command")
        end

        local pitch_deg = ahrs:get_pitch() * 180.0 / math.rad(180.0)
        if (pitch_deg > pitch_up_angle_threshold1) then
          stage = 6
          cruise_counter = 0
          gcs:send_text(0, "transition to stage6: Cruise to -X")
        end

      elseif (stage == 6) then  -- Stage6: cruise to -x
        cruise_counter = cruise_counter + 1

        if (cruise_counter > cruise_counter_max2) then
          stage = 7
          pitch_deg_prev = 1.0e8
          gcs:send_text(0, "transition to stage7: Pitch up")
        end

        -- send angle request
        if not (vehicle:set_target_angle_and_climbrate(0.0, cruise_pitch_angle2, yaw_target, 0.0, false, 0.0)) then
          gcs:send_text(0, "failed to execute angle command")
        end

      elseif (stage == 7) then  -- Stage6: pitch up and transition to cruise to X
        local roll_rate_rs = 0.0
        local pitch_rate_rs = pitch_up_rate2 * math.rad(180.0) / 180.0
        local yaw_rate_rs = 0.0
        local thrust = 0.0

        -- send rate request
        if not (vehicle:set_target_rate_and_thrust(roll_rate_rs, pitch_rate_rs, yaw_rate_rs, thrust)) then
          gcs:send_text(0, "failed to execute rate command")
        end

        local pitch_deg = ahrs:get_pitch() * 180.0 / math.rad(180.0)
        if (pitch_deg < 0.0 and pitch_deg > pitch_up_angle_threshold2 and pitch_deg > pitch_deg_prev) then
          stage = 4  -- X cruise
          cruise_counter = 0
          gcs:send_text(0, "transition to stage4: Cruise to X")
        end
        pitch_deg_prev = pitch_deg
      end

      -- Geo fence
      if stage >= 3 then
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          local alt = -vec_from_home:z()
          local north = vec_from_home:x()
          local east = vec_from_home:y()
          local yaw_target_rad = yaw_target * math.rad(180.0) / 180.0
          local x = north * math.cos(yaw_target_rad) + east * math.sin(yaw_target_rad)
          local y = -north * math.sin(yaw_target_rad) + east * math.cos(yaw_target_rad)
          if alt < 10 or 50 < alt or x < -10 or 40 < x or math.abs(y) > 10 then
            vehicle:set_mode(6)
            gcs:send_text(0, "Geo fence violation: transition to RTL")
            stage = -1
          end
        end
      end

    end
  end

  return update, 100
end

return update()
