PID_library

This library should in reality be called „PIVFF“ library, since it uses 4 types on controllers inside:
• P – proportional controller
• I – integral controllers
• V – velocity based controller
• FF – feed forward controller

It is recommended to use PIFF controller (with V (D) set to 0) for applications involving control of Airplanes.
 
One of the specialties of the library is that it has an input for attenuation which allows to change the sensitivity of the loop based on the speed of the airplane. It also allows the user to change parameters on the fly, to reset internal parameters or completely disable the whole loop
without having any issues after reinitialisation.




