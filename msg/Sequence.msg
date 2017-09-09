time start
time end

# pins and seq must be indexed the same, with pins[i] = p
# for each transition that should happen on digital pin p.

# Messages were nested before, with one pin per list of 
# Transistions, but rosserial could not generate appropriate
# C++ code to deserialize that type.
uint8[] pins
Transition[] seq
