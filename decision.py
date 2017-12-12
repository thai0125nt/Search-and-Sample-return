import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if len(Rover.nav_angles) ==0:
            Rover.nav_angles = np.zeros((1))
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                if Rover.vel >= Rover.max_vel: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if Rover.nav_angles_left is not None:
                    Rover.steer = Rover.nav_angles_left * 180/np.pi - 15
                if Rover.nav_angles_left is None:
                    Rover.steer = np.mean(Rover.nav_angles * 180/np.pi)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            if len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = 0
                Rover.steer = 0
                Rover.mode = 'stop'
            if Rover.throttle == Rover.throttle_set and np.abs(Rover.vel) <=0.1 and Rover.stop_time is None:
                Rover.stop_time = Rover.total_time
            if Rover.throttle == Rover.throttle_set and np.abs(Rover.vel) <=0.1 and Rover.stop_time is not None:                
                if Rover.total_time - Rover.stop_time >= 6:
                    Rover.mode = 'drive_back'
            if Rover.boo == 1:
                Rover.steer = -Rover.steer
                if Rover.total_time - Rover.stop_time >= 10:
                    Rover.boo = 0
                    Rover.stoptime = None
         # If we're already in "stop" mode then make different decisions
        if Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            Rover.stop_time = None
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            if Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # Could be more clever here about which way to turn
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        if Rover.mode == 'drive_back':
            Rover.throttle = -0.2
            Rover.brake = 0
            if Rover.nav_angles_left is not None:
                Rover.steer = Rover.nav_angles_left * 180/np.pi
            if Rover.nav_angles_left is None:
                Rover.steer = np.mean(Rover.nav_angles * 180/np.pi)
            if Rover.total_time - Rover.stop_time >=11 and Rover.vel <=-0.4 :#and Rover.total_time - Rover.stop_time <10:
                Rover.mode = 'forward'
                Rover.boo = 1
            if Rover.total_time - Rover.stop_time >=11 and Rover.vel >= -0.4 :
                    Rover.throttle = 0
                    Rover.steer = -15
                    if Rover.total_time - Rover.stop_time >=13.5:
                        Rover.mode = 'forward'
                        Rover.boo = 0
                        Rover.stop_time = None            
        if Rover.mode == 'picking_up':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if Rover.near_sample:
                Rover.send_pickup = True
                Rover.nav_angles_rock = None
            else:
                Rover.send_pickup = False
                Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    if Rover.nav_angles is None:
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = Rover.brake_set
    if Rover.nav_angles_rock is not None:
        if len(Rover.nav_angles_rock) >=1:
            
            Rover.steer = np.clip(np.mean(Rover.nav_angles_rock * 180/np.pi), -15, 15)
            if Rover.near_sample and not Rover.picking_up:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    
                    Rover.mode = 'picking_up'
 
    # If in a state where want to pickup a rock send pickup command
    
    
    return Rover

