

void pidhesap(void){
    float hata_roll;
    float hata_pitch;
    float hata_yaw;


    roll_ref = 0; // Derece
    pitch_ref = 0; //Derece
    yaw_ref = 0; //Derece 

    hata_roll = euler.x() - roll_ref ;//x, y ve z sensör konumuna göre değişecek
    pid_i_roll_cikti += pid_i_roll*hata_roll;
    if(pid_i_roll_cikti > pid_i_roll_max)pid_i_roll_cikti=pid_i_roll_max;
    else if (pid_i_roll < pid_i_roll_max*-1)pid_i_roll_cikti=pid_i_roll_max*-1;
    
    pid_roll_cikti = pid_p_roll*hata_roll;
    if(pid_p_roll_cikti > pid_p_roll_max)pid_p_roll_cikti=pid_p_roll_max;
    else if (pid_p_roll < pid_p_roll_max*-1)pid_p_roll_cikti=pid_p_roll_max*-1;

    pid_d_roll_cikti = hata_roll;   
    
    
    hata_pitch = euler.y() - pitch_ref;







}