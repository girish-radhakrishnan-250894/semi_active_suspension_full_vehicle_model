# semi_active_suspension_full_vehicle_model
 Full car with semi-active suspension system that controls damping cofficient and hence the sprung mass displacement of each corner

# RUNNING THE MODEL
a_entry_point.m                             -> As the name suggests, this is the entry script. Simply hitting PLAY inside this scrpt wil result running the observer simulation and plots the results
input_script.m                              -> To edit inputs
semi_active_suspension_full_vehicle.m       -> Wrapper function which is called by the ODE function. This function contains ESC algorithm. This function calls the core vehicle model from within
vehicle_model_fw_simplified.m               -> Core function containing the equations of motion of the vehicle model. This is function has only 1 role, accept some inputs and states, and calculate the forces and accelerations
