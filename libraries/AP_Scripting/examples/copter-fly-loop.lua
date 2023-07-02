-- command a Copter to takeoff and vertical loop fly
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 'takeoff_alt_above_home' meters
--    c) cruise to north direction with 'cruise_counter_max_north / 10' sec
--    d) pitch up with 'pitch_up_rate1' deg/sec
--    e) cruise to south direction with 'cruise_counter_max_south / 10' sec
--    f) pitch up with 'pitch_up_rate2' deg/sec
--    g) goto (c)

local takeoff_alt_above_home = 20  -- m
local copter_guided_mode_num = 4
local stage = 0
local cruise_speed = 10.0  -- m/sec
local cruise_counter = 0
local cruise_counter_max_north = 40  -- counts (10counts = 1sec)
local cruise_counter_max_south = 30  -- counts (10counts = 1sec)
local pitch_up_rate1 = 48.0  -- deg/sec
local pitch_up_rate2 = 180.0  -- deg/sec
local pitch_deg_prev = 1.0e8

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          stage = stage + 1
        end

      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = stage + 1
          gcs:send_text(0, "transition to stage2: Alt hold")
        end

      elseif (stage == 2) then      -- Stage2: altitude hold
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
            stage = stage + 1
            gcs:send_text(0, "transition to stage3: Cruise to North")
          end
        end

      elseif (stage == 3) then   -- Stage3: cruise to north
        cruise_counter = cruise_counter + 1

        local target_vel = Vector3f()
        target_vel:x(cruise_speed)
        target_vel:y(0.0)
        target_vel:z(0.0)

        if (cruise_counter > cruise_counter_max_north) then
          stage = stage + 1
          gcs:send_text(0, "transition to stage4: Pitch up")
        end

        -- send velocity request
        if not (vehicle:set_target_velocity_NED(target_vel)) then
          gcs:send_text(0, "failed to execute velocity command")
        end

      elseif (stage == 4) then  -- Stage4: pitch up and transition to cruise to south
        local roll_rate_rs = 0.0
        local pitch_rate_rs = pitch_up_rate1 * 3.14 / 180.0
        local yaw_rate_rs = 0.0
        local thrust = 1.0

        -- send rate request
        if not (vehicle:set_target_rate_and_thrust(roll_rate_rs, pitch_rate_rs, yaw_rate_rs, thrust)) then
          gcs:send_text(0, "failed to execute rate command")
        end

        local pitch_deg = ahrs:get_pitch() * 180.0 / 3.14
        if (pitch_deg > 25.0) then
          stage = stage + 1
          cruise_counter = 0
          gcs:send_text(0, "transition to stage5: Cruise to South")
        end

      elseif (stage == 5) then  -- Stage5: cruise to -x (NED)
        cruise_counter = cruise_counter + 1

        local target_vel = Vector3f()
        target_vel:x(-cruise_speed)
        target_vel:y(0.0)
        target_vel:z(0.0)

        if (cruise_counter > cruise_counter_max_south) then
          stage = stage + 1
          pitch_deg_prev = 1.0e8
          gcs:send_text(0, "transition to stage6: Pitch up")
        end

        -- send velocity request
        if not (vehicle:set_target_velocity_NED(target_vel)) then
          gcs:send_text(0, "failed to execute velocity command")
        end

      elseif (stage == 6) then  -- Stage6: pitch up and transition to cruise to north
        local roll_rate_rs = 0.0
        local pitch_rate_rs = pitch_up_rate2 * 3.14 / 180.0
        local yaw_rate_rs = 0.0
        local thrust = 0.0

        -- send rate request
        if not (vehicle:set_target_rate_and_thrust(roll_rate_rs, pitch_rate_rs, yaw_rate_rs, thrust)) then
          gcs:send_text(0, "failed to execute rate command")
        end

        local pitch_deg = ahrs:get_pitch() * 180.0 / 3.14
        if (pitch_deg < 0.0 and pitch_deg > -50.0 and pitch_deg > pitch_deg_prev) then
          stage = 3  -- north cruise
          cruise_counter = 0
          gcs:send_text(0, "transition to stage3: Cruise to North")
        end
        pitch_deg_prev = pitch_deg
      end
    end
  end

  return update, 100
end

return update()
