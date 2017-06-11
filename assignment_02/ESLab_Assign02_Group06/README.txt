********************************************************************************
||                ____                             ___   __                   ||
||               / ___|_ __ ___  _   _ _ __       / _ \ / /_                  ||
||              | |  _| '__/ _ \| | | | '_ \     | | | | '_ \                 ||
||              | |_| | | | (_) | |_| | |_) |    | |_| | (_) |                ||
||               \____|_|  \___/ \__,_| .__/      \___/ \___/                 ||
||                                    |_|                                     ||
********************************************************************************


===================
|| GROUP MEMBERS ||
================================================================================
                                                                              ||
NAME                      STUD.ID                                             ||
                                                                              ||
Carlo Delle Donne         4642718                                             ||
Dimitris Patoukas         4625943                                             ||
Vinay Pathi Balaji        4617363                                             ||
Ishu Goel                 4632699                                             ||
Nikolas Skartsilas        4631560                                             ||
Viktoria Mavrikopoulou    4628225                                             ||
                                                                              ||
================================================================================


======================
|| FOLDER STRUCTURE ||
================================================================================
                                                                              ||
+-- code                                                                      ||
|   |                                                                         ||
|   +-- dsp                     DSP side source files                         ||
|   |                                                                         ||
|   +-- gpp                     GPP side source files                         ||
|                                                                             ||
+-- report                                                                    ||
    |                                                                         ||
    +-- Group06_report.pdf      Assignment report in pdf format               ||
                                                                              ||
================================================================================


===================
|| INSTRUCTIONS  ||
================================================================================
                                                                              ||
Compile the code  --->  The code has to be compiled on the eslab server       ||
                        due to libraries and compilers dependencies:          ||
                                                                              ||
                        $ cd dsp                                              ||
                        $ make                                                ||
                        $ cd ../gpp                                           ||
                        $ make                                                ||
                                                                              ||
Run the code  ------->  Besides the 'car.avi' input video, copy the following ||
                        files to the BeagleBoard:                             ||
                                                                              ||
                        dsp/Release/pool_notify.out  (DSP executable)         ||
                        gpp/Release/meanShiftGroup06 (GPP executable)         ||
                                                                              ||
                        Then run:                                             ||
                                                                              ||
                        $ ./meanShiftGroup06 car.avi                          ||
                                                                              ||
                        Note that the DSP executable must not be passed as    ||
                        command line argument.                                ||
                                                                              ||
Important notes  ---->  By default, the application runs on both GPP and DSP. ||
                |       Nevertheless, the fastest implementation is the one   ||
                |       that runs on the GPP only. To test the latter, edit   ||
                |                                                             ||
                |       gpp/makefile                                          ||
                |                                                             ||
                |       remove the define -DDSP at line 92, recompile and run.||
                |                                                             ||
                 ---->  By default, the console output shows extensive timing ||
                        information. To suppress some of the output, edit     ||
                                                                              ||
                        gpp/makefile                                          ||
                                                                              ||
                        remove -DTIMING at line 91, recompile and run.        ||
                                                                              ||
================================================================================