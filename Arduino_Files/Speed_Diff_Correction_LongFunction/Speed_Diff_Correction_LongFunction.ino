void Mot_Diff_Correction(){

    Setpoint_sd = Lmot.get_Dn(myEnc_l.read()); // Set point is left motor difference between cureent enc reading and reference value
    Input_sd = Rmot.get_Dn(myEnc_r.read()); // Input is right motor difference between cureent enc reading and reference value
    Motor_Diff.Compute(); // Compute the PID output

    /*In the code below, all the conditions appear similar dues to sign conventions
     therefore, let's try to replace 4 checks with just 1*/

    bool delta_n1 = Setpoint_sd>0 & Input_sd>0; // Case when the robot is in +X region i.e. diff b/w enc_count_now and ref_enc_count is "+ve"
    bool delta_n2 = Setpoint_sd<0 & Input_sd<0; // Case when the robot is in -X region i.e. diff b/w enc_count_now and ref_enc_count is "-ve"
    
    /*If the robot is in +X region there can be 3 possibilities namely
    1) The robot has a +ve V_trans and is accelerating due to a command "go fwd" given
    2) The robot has a +ve V_trans and is de-celerating because it was pushed from the rest position or due to a "stop" command given after "go fwd"
    3) The robot has a -ve V_trans and is either returning to the origin or is braking after "stop" command 

    Else If the robot is in -X region there can be 3 possibilities namely
    1) The robot has a -ve V_trans and is accelerating due to a command "go bck" given
    2) The robot has a -ve V_trans and is de-celerating because it was pushed from the rest position or due to a "stop" command given after "go bck"
    3) The robot has a +ve V_trans and is either returning to the origin or is braking after "stop" command 
    */

    if (delta_n1){

// Case1: In this case both Output_lmot and Output_rmot are "-ve" and the robot is accelerating and is bent in +ve direction
      if (V_trans>0.0 & mode_now == "go fwd"){ 

        if (Output_sd>0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot+=Output_sd; // Output_lmot is -ve, so to decrease it, we have to "add" a +ve quantity
          Output_rmot-=Output_sd; // Output_rmot is -ve, so to increase it, we have to "subtract" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot+=Output_sd; // Output_lmot is -ve, so to increase it, we have to "add" a -ve quantity
          Output_rmot-=Output_sd; // Output_rmot is -ve, so to decrease it, we have to "subtract" a -ve quantity
        }
      }

//case 2: In this case both Output_lmot and Output_rmot are "+ve and the robot is de-celerating and is bent in -ve direction"
      else if (V_trans>0.0 & mode_now == "balance"){ 

        if (Output_sd>0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot-=Output_sd; // Output_lmot is +ve, so to decrease it, we have to "subtract" a +ve quantity
          Output_rmot+=Output_sd; // Output_rmot is +ve, so to increase it, we have to "add" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot-=Output_sd; // Output_lmot is +ve, so to increase it, we have to "subtract" a -ve quantity
          Output_rmot+=Output_sd; // Output_rmot is +ve, so to decrease it, we have to "add" a -ve quantity
        }
      }

/*case 3: In this case both Output_lmot and Output_rmot are "+ve" and the robot is either returning to 
vertical position or returning back to the origin & bent in -ve direction*/
      else if (V_trans<0.0 & mode_now == "balance"){ 

        if (Output_sd>0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot+=Output_sd; // Output_lmot is +ve, so to increase it, we have to "add" a +ve quantity
          Output_rmot-=Output_sd; // Output_rmot is +ve, so to decrease it, we have to "subtract" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot+=Output_sd; // Output_lmot is +ve, so to decrease it, we have to "add" a -ve quantity
          Output_rmot-=Output_sd; // Output_rmot is +ve, so to increase it, we have to "subtract" a -ve quantity
        }
      }               
    }

    else if (delta_n2){

// Case1: In this case both Output_lmot and Output_rmot are "+ve" and the robot is accelerating and is bent in -ve direction
      if (V_trans<0.0 & mode_now == "go bck"){ 

        if (Output_sd>0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot+=Output_sd; // Output_lmot is +ve, so to increase it, we have to "add" a +ve quantity
          Output_rmot-=Output_sd; // Output_rmot is +ve, so to decrease it, we have to "subtract" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot+=Output_sd; // Output_lmot is +ve, so to decrease it, we have to "add" a -ve quantity
          Output_rmot-=Output_sd; // Output_rmot is +ve, so to increase it, we have to "subtract" a -ve quantity
        }
      }

//case 2: In this case both Output_lmot and Output_rmot are "-ve and the robot is de-celerating and is bent in +ve direction"
      else if (V_trans<0.0 & mode_now == "balance"){ 

        if (Output_sd>0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot-=Output_sd; // Output_lmot is -ve, so to increase it, we have to "subtract" a +ve quantity
          Output_rmot+=Output_sd; // Output_rmot is -ve, so to decrease it, we have to "add" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot-=Output_sd; // Output_lmot is -ve, so to decrease it, we have to "subtract" a -ve quantity
          Output_rmot+=Output_sd; // Output_rmot is -ve, so to increase it, we have to "add" a -ve quantity
        }
      }

/*case 3: In this case both Output_lmot and Output_rmot are "-ve" and the robot is either returning to 
vertical position or returning back to the origin & bent in +ve direction*/
      else if (V_trans>0.0 & mode_now == "balance"){ 

        if (Output_sd>0.0){ // This means left motor is spinning faster, so reduce its speed and increase right motor speed
          Output_lmot+=Output_sd; // Output_lmot is -ve, so to decrease it, we have to "add" a +ve quantity
          Output_rmot-=Output_sd; // Output_rmot is -ve, so to increase it, we have to "subtract" a +ve quantity
        }

        else if (Output_sd<0.0){ // This means right motor is spinning faster, so reduce its speed and increase left motor speed
          Output_lmot+=Output_sd; // Output_lmot is -ve, so to increase it, we have to "add" a -ve quantity
          Output_rmot-=Output_sd; // Output_rmot is -ve, so to decrease it, we have to "subtract" a -ve quantity
        }
      }               
    } 
  // /*Correct for motor differences only when a command to go forward or backward is given*/
 //    if (V_trans>0.0 & mode_now == "go fwd" & mode_prev == "go fwd"){ // Robot is going forward, both Ouput_lmot & Ouput_rmot are -ve
 //      if (Output_sd>0.0){ // Left motor is faster, so increase right motor speed and decrease left motor speed
 //        Output_rmot-=Output_sd; // Output_rmot is -ve, so to increase it, we have to "subtract" a +ve quantity
 //        Output_lmot+=Output_sd; // Output_lmot is -ve, so to decrease it, we have to "add" a +ve quantity

 //      }
 //      else if (Output_sd<0.0){ // Right motor is faster, so increase left motor speed and decrease right motor speed
 //        Output_lmot+=Output_sd; // Output_lmot is -ve, so to increase it, we have to "add" a -ve quantity
 //        Output_rmot-=Output_sd; // Output_rmot is -ve, so to decrease it, we have to "subtract" a -ve quantity
 //      }
 //    }
 //    else if (V_trans<0.0 & mode_now == "go bck" & mode_prev == "go bck"){ // Robot is going backward, both Ouput_lmot & Ouput_rmot are +ve
 //      if (Output_sd>0.0){ // Right motor is faster, so increase left motor speed
 //        Output_lmot+=Output_sd; // Output_lmot is +ve, so to increase it, we have to "add" a +ve quantity
 //        Output_rmot-=Output_sd; // Output_rmot is +ve, so to decrease it, we have to "subtract" a +ve quantity

 //      }
 //      else if (Output_sd<0.0){ // Left motor is faster, so increase right motor speed
 //        Output_rmot-=Output_sd; // Output_rmot is +ve, so to increase it, we have to "subtract" a -ve quantity
 //        Output_lmot+=Output_sd; // Output_lmot is +ve, so to decrease it, we have to "add" a -ve quantity
 //      }
 //    }        
}

