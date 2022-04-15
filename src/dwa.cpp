#include<stdio.h>

DWA::DWA():private_nh("~"){
    private_nh.param("hz", hz, {10});
    
}