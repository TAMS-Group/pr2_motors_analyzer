pr2_motors_analyzer
======

The pr2 has the behavior that it throws the error "Safety Lockout: UNDERVOLTAGE" for the motors, when 
the emergency stop is pressed, no matter if it's the onboard or the wirless button.

This reposetory provides the behavior that this messages will not appear and the diagnostic status is OK, 
but the warning "Emergency stop is pressed".

---
