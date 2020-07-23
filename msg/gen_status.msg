uint64 timestamp				# time since system start (microseconds)
float32 genpower # power in W of generator side of system
float32 genenergy # energy (in kJ) consumed/produced by generator side of system
uint8 fuelpct # percentage of fuel remaining in the vehicle (based on genenergy / totalenergy)
float32 gencurrent # current in amps from generator side of system
uint8 genrunning # flag to say whether the generator motor is running
