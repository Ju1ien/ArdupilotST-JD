/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if OA_ENABLED == ENABLED

#include "AC_OA_Main.h"
/*
* Déplacer le contenu dans une librairie une fois le code validé
*/
                                  
// OA global variables
// General
static bool oa_is_used = true;  // ne pas exécuter le code autrement, à implementer plus tard
static int8_t oa_scheduler_step = 0;

// Gimbal & position type enum
enum oa_gimbal_type {
    PAN_TILT_2_AXIS = 0,                // Default gimbal type
    PAN_ONLY = 1,                       // TBD
    PAN_TILT_PAN = 2,                   // TBD + fixed + ...
};

enum oa_gimbal_position {
    ABOVE = 0,                          // Default gimbal position
    BELOW = 1,                          // TBD
    FRONT = 2,                          // TBD + rear + left + right
};

// Scan algorithms enum
enum scan_algo {
    NO_MAP_SCAN = 0,                    // no GPS, just aim in the moving direction from roll/pitch/yaw angles
    GENERAL_SCAN_NO_VEL = 1,            // when copter is loitering with no velocity, we scan everywhere because we don't know in which direction will be the next command.
                                        // a periodic scan of the top/bottom/front/... depending on the gimbal position is compulsory
    SCANNABLE_VERTICAL = 2,             // copter moving, the gimbal needs to scan mainly what's up/down (vertically)
    SCANNABLE_HORIZONTAL = 3,           // copter moving, the gimbal needs to scan mainly what's around (horizontally)
    UNSCANNABLE_Z_ANTICIPATE = 4,       // copter moving, gimbal unable to scan the relevant path. But we can still check in case of the vel.z changes 
                                        // (eg: gimbal above, copter moving forward down, we'll scan in the moving direction (forward) in case of the vel.z becomes positive the "new path" will be already scanned. We'll keep an eye as well on the roof (top)
};

// Earth Frame or Body Frame angle type used by the selected scan algo
enum oa_gimbal_angles_type {
    EF_ANGLES = 0,                          // Default gimbal position
    BF_ANGLES = 1,                          // TBD
};



oa_gimbal_type  gimbal_type = PAN_TILT_2_AXIS;      // Could be set as param if several gimbal types are defined
oa_gimbal_position gimbal_position = ABOVE;         // same for param opt

// Gimbal - 2 axis Pan+Tilt
static bool reverse_gimbal;                 // Reverse the gimbal tilt/pan in ordert to cover 360° pan without hurting the servo/gimbal limits
static float Ay_max_plus_y_delta;           // Ay_max is used to change scan algo. Ay_max = Ay_max_plus_y_delta - y_delta; we compute the constant Ay_max_plus_y_delta term once at init
//static float ef_gimbal_Ay=0, ef_gimbal_Az;   // TO-DO : suppr ces variables
static float des_ef_gimbal_Ay=0, real_ef_gimbal_Ay=0;     // Earth frame gimbal angles. Rad for computations [-PI;PI]. desired is demanded by scan algo. real is corrected according to the gimbal mechanical limits. real angle is used to update the mapping
static float des_ef_gimbal_Az=0, real_ef_gimbal_Az=0;     // Earth frame gimbal angles. Rad for computations [-PI;PI]. desired is demanded by scan algo. real is corrected according to the gimbal mechanical limits. real angle is used to update the mapping
static float bf_gimbal_Ay=0, last_bf_gimbal_Ay=0;            // Body frame gimbal angles. Rad for computations [-PI;PI]. But passed as deg for gimbal control
static float bf_gimbal_Az=0, last_bf_gimbal_Az=0;            // Body frame gimbal angles. Rad for computations [-PI;PI]. But passed as deg for gimbal control
static uint32_t last_gimbal_control_time;
static int16_t last_scan_algo=100;          // 100 means nothing, that's required to init properly the first scan_algo
static int16_t scan_step=0;
static bool use_ef_or_bf_gimbal_angles;        // EF = 0, BF = 1
static bool gimbal_control_lock = false;    // Prevent the gimbal to receive a new control command until de lrf didn't return the measure of the current position
        
static float uav_y_reduced_inclination;    // in rad. UAV inclination reduced to Y axis
//static float uav_x_reduced_inclination;    // in rad. UAV inclination reduced to X axis
static float S_proj;                       // in cm. UAV size (* safety factor) projected on the tilt axis
static float W_proj;                       // in cm. UAV size (* safety factor) projected on the pan axis
static int8_t y_sign;                       //used for scan algo. may depend on the gimbal position
static int8_t z_sign;                       //used for scan algo.
static float gimbal_Az_scan_offset;         // used for scan algo
static float gimbal_Ay_scan_offset;         // used for scan algo
static int8_t pitch_count;                  // used for scan algo
static bool inc_offset;                     // used for scan algo
  
// mapping
static bool oa_initialized = false;
static bool oa_use_map = true;
static char oa_map[OA_MAP_SIZE_X][OA_MAP_SIZE_Y][OA_MAP_SIZE_Z];
static Vector3f oa_map_origin;  // origine du volume de mapping. ce point se déplace avec le drone en fonction de l'offset calculé
static Vector3f oa_map_origin_to_copter_offset; // calculer cet offset en fonction de la vitesse du drone de façon à tirer partie au maximum du mapping 3D tout en limitant les besoins en ressource RAM/CPU
static Vector3f oa_last_copter_pos; // Inutile, à voir??? position du drone lorsque le mapping a été recallé pour la dernière fois de façon à obtenir copter_pos = map_origin+map_to_copter_offset. Quand copter_pos >= last_copter_pos +/- map_res, on décale le tableau d'une case dans l'axe concerné
static int oa_min_dist = constrain_int16(OA_MIN_DIST, COPTER_DIAMETER/2+20, 300); // in cm. ensure we are in decent range: from prop guard to 3m
static float oa_copter_size_factor = constrain_float(COPTER_SIZE_SAFETY_FACTOR, 1.0f, 4.0f);
static int16_t lrf_dist;   // measure from lrf sensor (cm)
static int16_t safe_dist;  // safe distance in the moving direction. (cm)
static float stopping_dist; // in cm. distance between copter position and stopping point.updated by oa_update_map_from_copter_pos_and_vel().
static Vector3f copter_map_index;  // index of the copter position in the mapping table. With a float value we are using the exact position of the copter in the cell.
static bool object_detected;

// to be declared unstatic later and passed though function param
// commenté pour debug static const Vector3f& oa_copter_pos = inertial_nav.get_position();      // cm from home. Current copter pos
//A suppr car vel anticipée sera utilisée. commenté pour debug static const Vector3f& vel = inertial_nav.get_velocity();                // cm/s. Current copter vel
static Vector3f oa_copter_pos;// = {0.0f,0.0f,0.0f}; //debug
static Vector3f vel;    //debug
static float vel_xy;
    

    //OA functions

// oa_init - Initialise les variables et le mapping 3D
// Voir pour remplacer par un init OA général qui calcule toutes les variables intermédiaires
static void oa_init()
{
    int x,y,z;
    float Kd;
    
    switch (gimbal_type){
        case PAN_TILT_2_AXIS:
            reverse_gimbal = false; // Init initial gimbal mode
            // ajouter ici les calculs des constantes et réaliser les vérif de dmin et kd
            // on vérifie que oa_copter_size_factor est compatible avec la distance minimale d'approche oa_min_dist
            Kd = 0.7*oa_min_dist*sin(MAX_SCAN_PAN_ANGLE/2)/COPTER_DIAMETER;
            if(Kd<1){   // si la distance est trop petite, la redéfinir.
                oa_copter_size_factor = 1;
                oa_min_dist = COPTER_DIAMETER/(0.7*sin(MAX_SCAN_PAN_ANGLE/2));
            }
            
            switch(gimbal_position){
                case ABOVE:
                    // compute here Ay_max_plus_y_delta constant term once
                    Ay_max_plus_y_delta = acosf(COPTER_DIAMETER*oa_copter_size_factor/(2*oa_min_dist*sinf(MAX_SCAN_PAN_ANGLE/2)));
                    y_sign = 1;
                    // to-do voir pour gérer une nacelle sous le drone, cet angle doit etre negatif donc
                    break;
                case BELOW:
                    y_sign = -1;
                    // compute here Ay_max_plus_y_delta constant term once
                    // a voir s'il suffit d'un sign neg devant   Ay_max_plus_y_delta = acosf(COPTER_DIAMETER*oa_copter_size_factor/(2*oa_min_dist*sinf(MAX_SCAN_PAN_ANGLE/2)));
                    // to-do voir pour gérer une nacelle sous le drone, cet angle doit etre negatif donc
                    break;
            }
            
            break;
    }

    // init OA map
    // Init mapping table . Values: 0=No object, 1=Not sure-to check, 2=Object detected
    // Every value is set to "unchecked"
    for (x=0; x<OA_MAP_SIZE_X; x++){
        for (y=0; y<OA_MAP_SIZE_Y; y++){
            for (z=0; z<OA_MAP_SIZE_Z; z++){
                oa_map[x][y][z] = 1;
            }
        }
    }
    
    // init gimbal control variable
    camera_mount.set_mode(MAV_MOUNT_MODE_OA_BYPASS);	
    gimbal_control_lock = false;
    last_gimbal_control_time=millis()+2000;    // Set it to 2s in the future to let at least 2s for the gimbal and lrf to initialize properly on startup before getting the lrf measure and update the map with it
    
    // initialiser la position du drone et celle du tableau de mapping
    // Si le GPS n'est pas disponible, le mapping n'est pas utilisable
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D){
        oa_last_copter_pos = oa_copter_pos;     // Voir si utile ...? Set the previous copter pos as the current one to avoid moving the MAP worthless
    }
    
    //init oa_scheduler
    oa_scheduler_step = 0;
    
    //TO-DO : initialiser le lidar pour lui permettre un taux de refresh rep rate à 100Hz
}


// oa_run - Runs the object avoidance whole code. Cela se traduit globalement par X étapes
// TO-DO.. revoir ces étapes il y en a davantage cf le code ci-apres
// 1: Identification de la trajectoire à partir du vecteur vitesse du drone (+ acc pour anticiper?)
// 2: Définition du "tunel" (succession de fenetres) emprunté par le drone
// 3: Vérifier pour chacune de ces fenetres l'absence d'objet depuis le mapping 3D
// 4: Scanner les zones indéterminées (algo de scan fixe ou adaptatif -on scanne les zones incertaines en priorité-)
// 5: Restreindre les vitesses et position du drone en fonction du résultat (voir si création d'un mode de vol spécial pour commencer basé sur loiter)
static void oa_run()
{
    // variables declaration
    float Ay;                           // EF copter velocity vector rising angle. in Rad
    float Az;                           // EF copter velocity vector yaw angle. in Rad
    int16_t scan_algo_g1;                      // Scan algorithm selected for gimbal 1
    uint32_t dt;    

    //test variables and conditions
    bool test_rc5_1, test_rc5_2, test_rc5_3, test_rc5_4, test_rc5_5, test_rc5_6;
    bool test_rc6_1, test_rc6_2, test_rc6_3;
    bool test_rc7_1, test_rc7_2, test_rc7_3;
    bool test_rc8_1, test_rc8_2, test_rc8_3, test_rc8_g;
    bool debug_always_true = 1; 
    uint32_t test_dt_1;
    uint32_t test_dt_2;
    uint32_t test_dt_3;
    uint32_t test_dt_4;
    uint32_t test_dt_5;
    
    // test conditions
    test_rc5_1 = (g.rc_5.control_in<100);
    test_rc5_2 = ((g.rc_5.control_in>100) && (g.rc_5.control_in<300));
    test_rc5_3 = ((g.rc_5.control_in>300) && (g.rc_5.control_in<500));
    test_rc5_4 = ((g.rc_5.control_in>500) && (g.rc_5.control_in<700));
    test_rc5_5 = ((g.rc_5.control_in>700) && (g.rc_5.control_in<900));
    test_rc5_6 = (g.rc_5.control_in>900);
    test_rc6_1 = ((g.rc_6.control_in>100) && (g.rc_6.control_in<400));
    test_rc6_2 = ((g.rc_6.control_in>401) && (g.rc_6.control_in<700));
    test_rc6_3 = ((g.rc_6.control_in>701) && (g.rc_6.control_in<1001));
    test_rc7_1 = ((g.rc_7.control_in>100) && (g.rc_7.control_in<400));
    test_rc7_2 = ((g.rc_7.control_in>401) && (g.rc_7.control_in<700));
    test_rc7_3 = ((g.rc_7.control_in>701) && (g.rc_7.control_in<1001));
    test_rc8_1 = ((g.rc_8.control_in>100) && (g.rc_8.control_in<400));
    test_rc8_2 = ((g.rc_8.control_in>401) && (g.rc_8.control_in<700));
    test_rc8_3 = ((g.rc_8.control_in>701) && (g.rc_8.control_in<1001));
    test_rc8_g = (g.rc_8.control_in>100);
    
    //simulate vel vector debug
    //0-1000 -> -20+20 scaling
    vel.x = (float)(g.rc_6.control_in-500)*0.04f;
    vel.y = (float)(g.rc_7.control_in-500)*0.04f;
    vel.z = (float)(g.rc_8.control_in-500)*0.04f;
    
    // Enable/Disable OA
    if(!oa_is_used){
        oa_initialized = false;
        return;
    }
       
   //TO-DO: vérifier que ces lignes peuvent bien rester commentées.
   // Les références étant faites à la déclaration des variables, pas besoin de les redéfinir ici.
    //oa_copter_pos = inertial_nav.get_position();
    //vel = inertial_nav.get_velocity();  //TO-DO. Voir pour utiliser desired_vel à la place
    
    if(!oa_initialized){
        
        if (test_rc5_2){                                //debug
        test_dt_1 = micros();                           //debug
        
        oa_init();
        oa_initialized = true;
        return;     // init takes much more time than allowed so return immediatelly after init.
        
        test_dt_1 = micros() - test_dt_1;               //debug
        cliSerial->printf_P(PSTR("oa_init: %ld µs\n"),  //debug
					test_dt_1);                         //debug
        }
    }
    
    // Si le GPS n'est pas disponible, le mapping n'est pas utilisable on se contentera de scanner autour de l'axe directeur
    // en enregistrant uniquement la distance min sure, distance min avec objet
    if ((gps.status() >= AP_GPS::GPS_OK_FIX_2D)|| (debug_always_true)){   //debug suppr tjours vrai
        if(!oa_use_map){
            //oa_init(); A voir si nécessaire. le problème c'est qu'on RAZ tout le mapping même sur une micro perte du signal, c dommage!
            oa_use_map = true;
        }
    }else{
        oa_use_map = false;
    }
    
    // TO-DO
    // Gérer en switch/case avec des taux de refresh différents en fonction des perf requises par le code
    
    // 0-IDENTIFY MOVING DIRECTION AND ANGLES RELATED TO EARTH.
    // voir pour utiliser desired_vel si en loiter... ANTICIPATION du mouvement
    // ou calculer cette vitesse à partir des accélérations et/ou des commandes de l'utilisateur
    vel_xy = pythagorous2(vel.x, vel.y);
    Az = fast_atan2(vel.y, vel.x);                      // Rad value [-PI;PI]
    Ay = fast_atan2(vel.z, vel_xy); // Rad value [0-PI/4;PI+PI/4] 0->180 +/- Max_Copter_Angles. faux, on peut avoir -pi/2 si on descend uniquement
    // Attention pour Ay borner les valeurs car si négatives, le capteur ne pourra pas vérifier l'absence d'objets. Utiliser un sonar sous le drone plutot
    // faire un constrain ou utiliser cette valeur pour déterminer si l'utilisation du mapping est possible et l'algo de scan approprié
    
    // Identify path direction and window + copter attitude
    uav_y_reduced_inclination = ahrs.pitch*cosf(wrap_PI(ahrs.yaw-Az)) + ahrs.roll*sinf(wrap_PI(ahrs.yaw-Az));     // in rad. UAV inclination reduced to Y axis
    //uav_x_reduced_inclination = -ahrs.pitch*sinf(wrap_PI(ahrs.yaw-Az)) + ahrs.roll*cosf(wrap_PI(ahrs.yaw-Az));    // Not used, to comment... in rad. UAV inclination reduced to X axis
    S_proj = oa_copter_size_factor*(fabs(COPTER_HEIGHT*cosf(wrap_PI(Ay-uav_y_reduced_inclination)))+fabs(COPTER_DIAMETER*sinf(wrap_PI(Ay-uav_y_reduced_inclination)))); // in cm. UAV size projected on the tilt axis
    W_proj = oa_copter_size_factor*COPTER_DIAMETER;   // in cm. UAV size projected on the pan axis

    cliSerial->printf_P(PSTR("vel.x = %f, vel.y = %f, vel.z = %f\n"),      //debug
                                vel.x,
                                vel.y,
                                vel.z);
    cliSerial->printf_P(PSTR("Az = %f, Ay = %f\n"),      //debug
                                Az,
                                Ay);
                                
    /*cliSerial->printf_P(PSTR("S_proj = %f, W_proj = %f, Y = %f\n"),      //debug
                                S_proj,
                                W_proj,
                                uav_y_reduced_inclination);*/
                                
    cliSerial->printf_P(PSTR("Yaw = %ld cdeg, Pitch = %ld cdeg, Roll = %ld cdeg\n"),      //debug
                                ahrs.yaw_sensor,
                                ahrs.pitch_sensor,
                                ahrs.roll_sensor);
                                
    
    // 1-UPDATE MAP FROM COPTER NEW POSITION
    if((oa_use_map) || (debug_always_true)){                       //debug
        if (test_rc5_4){                                    //debug
        test_dt_2 = micros();                               //debug
        
        oa_update_map_from_copter_pos_and_vel();
        
        test_dt_2 = micros() - test_dt_2;                   //debug
        cliSerial->printf_P(PSTR("oa_update_map_from_copter_pos_and_vel: %ld µs\n"),      //debug
					test_dt_2);                             //debug
        }
    // Update copter map index (x/y/z index of the copter in mapping table)
        /*copter_map_index.x = round(floor(oa_copter_pos.x/OA_MAP_RES) - oa_map_origin.x/OA_MAP_RES);
        copter_map_index.y = round(floor(oa_copter_pos.y/OA_MAP_RES) - oa_map_origin.y/OA_MAP_RES);
        copter_map_index.z = round(floor(oa_copter_pos.z/OA_MAP_RES) - oa_map_origin.z/OA_MAP_RES);*/
        copter_map_index.x = (oa_copter_pos.x - oa_map_origin.x)/OA_MAP_RES;
        copter_map_index.y = (oa_copter_pos.y - oa_map_origin.y)/OA_MAP_RES;
        copter_map_index.z = (oa_copter_pos.z - oa_map_origin.z)/OA_MAP_RES;
        
        /*cliSerial->printf_P(PSTR("copter_map_index.x = %f, copter_map_index.y = %f, copter_map_index.z = %f\n"),      //debug
        copter_map_index.x,
        copter_map_index.y,
        copter_map_index.z);
        
        cliSerial->printf_P(PSTR("oa_copter_pos.x = %f, oa_copter_pos.y = %f, oa_copter_pos.z = %f\n"),      //debug
        oa_copter_pos.x,
        oa_copter_pos.y,
        oa_copter_pos.z);
        
        cliSerial->printf_P(PSTR("oa_map_origin.x = %f, oa_map_origin.y = %f, oa_map_origin.z = %f\n"),      //debug
        oa_map_origin.x,
        oa_map_origin.y,
        oa_map_origin.z);*/
  
    // 2-UPDATE MAP WITH NEW LRF READ
    // Check if servo had enough time to reach it's target control angles
        if (test_rc5_5){                                    //debug
        test_dt_3 = micros();                               //debug
        
        //to-do: optimiser le sequencement de la mesure et maj du mapping, cf ici
        // https://groups.google.com/forum/#!topic/pulsedlight3d/Xi3AjrWUE50
        
        dt = millis() - last_gimbal_control_time; // in ms. if OA in lib, use uint32_t now = hal.scheduler->millis() instead
        if((fabs(bf_gimbal_Ay - last_bf_gimbal_Ay)*1000/SERVO_TURN_RATE < dt) && (fabs(bf_gimbal_Az - last_bf_gimbal_Az)*1000/(SERVO_TURN_RATE*2) < dt)){  // ajouter un coeff de sécurité ET voir comment le dialogue avec le lidar permet de valider une lecture seulement aaprès le bon positionnement du servo
            //TO-DO: attention, la commande I2C d'acquisition de mesure est envoyée automatiquement à la fin der la lecture précédente, on ne va donc pas lire la bonne valeur mais certainement une valeur d'une position intermédiaire. revoir la librairie rangefinder en conséquent
            if(oa_lrf_read()){  // we wait as well for a new read of the LRF before updating the map
                if(lrf_dist != 0) oa_update_map_from_lrf_read();    // update map only if the lrf measure is correct - to avoid fake measures of copter prop for example
                last_bf_gimbal_Ay = bf_gimbal_Ay;     
                last_bf_gimbal_Az = bf_gimbal_Az;  
                gimbal_control_lock = false;
            }
        }
        
        test_dt_3 = micros() - test_dt_3;                   //debug
        cliSerial->printf_P(PSTR("oa_update_map_from_lrf_read: %ld µs\n"),      //debug
					test_dt_3);                             //debug
        }
        
    // 4-IDENTIFY PATH AND COPTER ATTITUDE
    // Groupé directement avec 5
    // 5-CHECK IF PATH IS FREE FROM ANY OBJECT
        // Check path windows from map
        
        if (test_rc5_6){                                    //debug
        test_dt_4 = micros();                               //debug
        
        safe_dist = oa_check_object_in_map(Ay, Az); // Ne pas exécuter si vel est nul (division par 0 autrement)
        //to-do : voir pour bufferiser les 10/20/30? dernières lectures du capteur suivant l'algo de scan (SCANNABLE_HORIZONTAL/VERTICAL)
        //et en prendre la minimale. Si le safe_dist retourné par oa_check_object_in_map() est lié à un manque de scan et non à une détection d'object
        //prendre le max du buffer et du mapping pour se donner plus de marge.
        // normalement ça devrait être bon car vecteur vitesse est anticipé, donc les scans bufferisés sont l'image de ce qu'il y a sur la trajectoire a l'instant t

        test_dt_4 = micros() - test_dt_4;                   //debug
        cliSerial->printf_P(PSTR("Safe dist = %d cm\n"),      //debug
                   safe_dist);
    
        cliSerial->printf_P(PSTR("oa_check_object_in_map: %ld µs\n"),      //debug
					test_dt_4);                             //debug
        }
        
        
    }else{
        if(oa_lrf_read()){
            safe_dist = lrf_dist;
            //to-do: voir plus haut si utilisation buffer
            //eg: safe_dist = max(lrf_dist, (min de mon tableau buffer)) ou plutot dans ce cas seul le min du tableau buffer.
            gimbal_control_lock = false;    // unlock gimbal control once measurement read
        }
        // do nothing? just target the lrf in the moving direction with a std scan algo  
        // We can't update neither use map so we will point the laser on our moving direction and use a special scan algo.        
    }
    
    if(!gimbal_control_lock){
        // 6-SELECT SCAN ALGO FROM THE COPTER ATTITUDE
        test_dt_4 = micros();                               //debug

        scan_algo_g1 = oa_select_scan_algo(Ay);

        /*cliSerial->printf_P(PSTR("scan_algo_g1: %ld \n"),      //debug
                scan_algo_g1);                             //debug
                */
        
        // 7-RUN THE SELECTED SCAN ALGO
        //TO-DO: décommenter les 3 lignes suivantes, commentées pour valider compilateur
        //if(gimbal_control_lock) ... //on développe la suite uniquement si la nacelle a été libérée. autrement cela n'est pas nécessaire
        //ef_gimbal_Ay = to define;       //Rad for computations. But Deg EF control_angles for gimbal stabilized tilt
        //ef_gimbal_Az = to define;       //Rad for computations. But Deg EF control_angles for gimbal stabilized pan
        if (test_rc5_5){                                    //debug
        oa_run_scan_algo(scan_algo_g1, Ay, Az);
        }
        // 8-UPDATE GIMBAL EF CONTROL ANGLES
        oa_gimbal_control(scan_algo_g1, Az);
        //to-do : revoir les paramètres de cette fonction car le scan algo n'est plus utilisé, Az à voir aussi...
        
        test_dt_4 = micros() - test_dt_4;                   //debug
        cliSerial->printf_P(PSTR("select_sacn_algo + run + gimbal_control: %ld µs\n"),      //debug
					test_dt_4);                             //debug
        
        
        // Lancer un chrono pour savoir depuis combien de temps l'ordre a été envoyé aux servo et ainsi évaluer leur position d'après leur datasheet (°/s)
        // ou alors brider le pas d'angle défini au servo pour faire en sorte qu'il ait atteint sa consigne entre 2 cycles.
        // par exemple last_gimbal_control_time = now à voir sur poshold par ex
        last_gimbal_control_time = millis();
        gimbal_control_lock = true; // we lock the gimbal control while we've not read the lrf measure of the current angles control
        }

    // TEST AND DEBUG ONLY
    //Display intermediary results
    //cliSerial->printf_P(PSTR("hello"));
    //inside of  a lib you can use hal.console->printf_P(PSTR("hello"));
    /*cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                        (unsigned)perf_info_get_num_long_running(),
                        (unsigned)perf_info_get_num_loops(),
                        (unsigned long)perf_info_get_max_time());*/
    //gcs_send_text_P(SEVERITY_HIGH,PSTR("test mess1")); // ok dans onglet mess, la console cliserial suffit


}

// oa_update_map_from_copter_pos_and_vel - Updates OA_Map from the new copter position and its velocity vector (in order to have more map head romm in the moving direction)
static void oa_update_map_from_copter_pos_and_vel()
{ 
    Vector3f oa_new_map_origin;
    Vector3f stopping_point;    
    Vector3f tmp_stopping_dist;    
    
    // Update stopping point from copter pos and velocity
    //commenté pour debug pos_control.get_stopping_point_xy(stopping_point);
    //commenté pour debug pos_control.get_stopping_point_z(stopping_point);
    
    //simulate stopping point debug
    //0-1000 -> -1000+1000 scaling
    /*stopping_point.x = (float)(g.rc_6.control_in-500)*2.0f;
    stopping_point.y = (float)(g.rc_7.control_in-500)*2.0f;
    stopping_point.z = (float)(g.rc_8.control_in-500)*2.0f;*/
    
    stopping_point.x = 0;  //debug
    stopping_point.y = 0;//debug
    stopping_point.z = 0;//debug
    
    // update stopping distance
    tmp_stopping_dist = stopping_point - oa_copter_pos;
    stopping_dist = tmp_stopping_dist.length();
    /*
    cliSerial->printf_P(PSTR("stopping_point.x = %f, stopping_point.y = %f, stopping_point.z = %f, stopping_dist = %f\n"),      //debug
            stopping_point.x,
            stopping_point.y,
            stopping_point.z,
            stopping_dist);
    */
    
    
    // We place that stopping point at the center of the MAP to get the most head room
    oa_new_map_origin.x = OA_MAP_RES * floor(stopping_point.x/OA_MAP_RES - OA_MAP_SIZE_X/2);
    oa_new_map_origin.y = OA_MAP_RES * floor(stopping_point.y/OA_MAP_RES - OA_MAP_SIZE_Y/2);
    oa_new_map_origin.z = OA_MAP_RES * floor(stopping_point.z/OA_MAP_RES - OA_MAP_SIZE_Z/2);
    /*       
    cliSerial->printf_P(PSTR("oa_copter_pos.x = %f, oa_copter_pos.y = %f, oa_copter_pos.z = %f\n"),      //debug
            oa_copter_pos.x,
            oa_copter_pos.y,
            oa_copter_pos.z);
    */
    // Ensure copter is still in the MAP (stopping point offset could be too high due to inapropriate copter vel)
    // Attention, de cette façon on peut placer le drone dans un des coins du mapping ce qui rend quasi impossible la vérification des vecteurs parallèle au vecteur vitesse drone cra leur point d'origine serait en-dehors du mapping
    // donc il faudrait peut etre se laisser une case a minima derrière et tolérer 2 ou 3 itérations hors du mapping avant de sortir de la boucle for lors de la vérif (check object in map) dans le cas ou le point d'origine est hors du cadre
    // ou alors borner la position du drone dans un sphere et non dans le cube de mapping.
    if(oa_copter_pos.x < oa_new_map_origin.x) oa_new_map_origin.x = OA_MAP_RES * floor(oa_copter_pos.x/OA_MAP_RES);
    if(oa_copter_pos.x > oa_new_map_origin.x+OA_MAP_SIZE_X*OA_MAP_RES) oa_new_map_origin.x = OA_MAP_RES * (floor(oa_copter_pos.x/OA_MAP_RES) - (OA_MAP_SIZE_X-1));
    if(oa_copter_pos.y < oa_new_map_origin.y) oa_new_map_origin.y = OA_MAP_RES * floor(oa_copter_pos.y/OA_MAP_RES);
    if(oa_copter_pos.y > oa_new_map_origin.y+OA_MAP_SIZE_Y*OA_MAP_RES) oa_new_map_origin.y = OA_MAP_RES * (floor(oa_copter_pos.y/OA_MAP_RES) - (OA_MAP_SIZE_Y-1));
    if(oa_copter_pos.z < oa_new_map_origin.z) oa_new_map_origin.z = OA_MAP_RES * floor(oa_copter_pos.z/OA_MAP_RES);
    if(oa_copter_pos.z > oa_new_map_origin.z+OA_MAP_SIZE_Z*OA_MAP_RES) oa_new_map_origin.z = OA_MAP_RES * (floor(oa_copter_pos.z/OA_MAP_RES) - (OA_MAP_SIZE_Z-1));
    /*
    cliSerial->printf_P(PSTR("oa_new_map_origin.x = %f, oa_new_map_origin.y = %f, oa_new_map_origin.z = %f\n"),      //debug
            oa_new_map_origin.x,
            oa_new_map_origin.y,
            oa_new_map_origin.z);
    */
    
    // Move map table if new map_origin is different from the previous one
    oa_move_map(oa_new_map_origin);
    
    // Set the new map origin
    oa_map_origin = oa_new_map_origin;
    
    // si on décale, alors on redéfini l'ancienne pos copter
    //pour chaque axe, ex oa_last_copter_pos.x = oa_copter_pos.x; 
}

// oa_move_map - Move the values of the map table according to the new map origin.
static void oa_move_map(Vector3f& new_map_origin)
{
    int offset_x = (int)round((new_map_origin.x-oa_map_origin.x)/OA_MAP_RES);
    int offset_y = (int)round((new_map_origin.y-oa_map_origin.y)/OA_MAP_RES);
    int offset_z = (int)round((new_map_origin.z-oa_map_origin.z)/OA_MAP_RES);    
    
    //simulate offsets
    //0 +1000 -> 0 +20 scaling
    offset_x = offset_y = offset_z = g.rc_6.control_in/50;  //debug
    
    /*cliSerial->printf_P(PSTR("offset_x = %d, offset_y = %d, offset_z = %d\n"),      //debug
        offset_x,
        offset_y,
        offset_z);*/
    //debug verif routine
    //on place une valeur repère au centre du mapping puis à la fin du déplacement tableau on recherche la position de ce repère
    //oa_map[OA_MAP_SIZE_X/2][OA_MAP_SIZE_Y/2][OA_MAP_SIZE_Z/2] = 5; //debug
    
    if(offset_x==0 && offset_y==0 && offset_z==0) return;    //If nothing to move, just return

    int x,y,z,x_init,y_init,z_init,x_stop,y_stop,z_stop,inc_x,inc_y,inc_z,prev_x,prev_y,prev_z;
    bool out_of_map_x, out_of_map_y, out_of_map_z;
    /*int inc_x = sgn(offset_x);
    int inc_y = sgn(offset_y);
    int inc_z = sgn(offset_z);   */ 
    
    if (offset_x<0){
        inc_x = -1;
        x_init = OA_MAP_SIZE_X-1;
        x_stop = -1;
    }else{
        inc_x = 1;
        x_init = 0;
        x_stop = OA_MAP_SIZE_X;
    }
    
    if (offset_y<0){
        inc_y = -1;
        y_init = OA_MAP_SIZE_Y-1;
        y_stop = -1;
    }else{
        inc_y = 1;
        y_init = 0;
        y_stop = OA_MAP_SIZE_Y;
    }
    
    if (offset_z<0){
        inc_z = -1;
        z_init = OA_MAP_SIZE_Z-1;
        z_stop = -1;
    }else{
        inc_z = 1;
        z_init = 0;
        z_stop = OA_MAP_SIZE_Z;
    }

    int test_map_1=0, test_map_move_pt=0;//debug
    
    x = x_init;
    while(x!=x_stop){
        prev_x = x+offset_x;
        out_of_map_x = (prev_x<0)||(prev_x>=OA_MAP_SIZE_X);
        
        /*cliSerial->printf_P(PSTR("x = %d, prev_x = %d, out_of_map_x = %d\n"),      //debug
                x,
                prev_x,
                out_of_map_x);*/

                
        y = y_init;                
        while(y!=y_stop){
            prev_y = y+offset_y;
            out_of_map_y = (prev_y<0)||(prev_y>=OA_MAP_SIZE_Y);
            z = z_init;
            while(z!=z_stop){
                prev_z = z+offset_z;
                out_of_map_z = (prev_z<0)||(prev_z>=OA_MAP_SIZE_Z);
                if(out_of_map_x || out_of_map_y || out_of_map_z){
                    oa_map[x][y][z] = 1;    //1 = unchecked(new) area
                    //test_map_1++; //debug
                }else{
                    oa_map[x][y][z] = oa_map[prev_x][prev_y][prev_z];
                    
                    //test_map_move_pt++;//debug
                    /*//debug
                    if(oa_map[prev_x][prev_y][prev_z]==5){
                    cliSerial->printf_P(PSTR("new_ref_map_x = %d, new_ref_map_y = %d, new_ref_map_z = %d\n"),      //debug
                        x,
                        y,
                        z);
                    }*/
                    
                    
                    
                }
                
                /*cliSerial->printf_P(PSTR("oa_map_x = %d, oa_map_x = %d, oa_map_x = %d\n"),      //debug
                        x,
                        y,
                        z);*/
                
                
                z += inc_z;
            }
            y += inc_y;
        }
        x += inc_x;
    }
    /*
    cliSerial->printf_P(PSTR("test_map_1 = %d, test_map_move_pt = %d\n"),      //debug
                        test_map_1,
                        test_map_move_pt);
    */
       
    /*
    //debug verif routine
    //on place une valeur repère au centre puis à la fin du déplacement tableau on recherche la position de ce repère
    for (x=0; x<OA_MAP_SIZE_X; x++){
        for (y=0; y<OA_MAP_SIZE_Y; y++){
            for (z=0; z<OA_MAP_SIZE_Z; z++){
                if(oa_map[x][y][z] == 5){
                    cliSerial->printf_P(PSTR("ref_map_x = %d, ref_map_y = %d, ref_map_z = %d\n"),      //debug
                        x,
                        y,
                        z);
                    return;
                }
            }
        }
    }*/
    
    
}

// oa_lrf_read - Read and filters the RangeFinder measure to get the best performances
static bool oa_lrf_read()
{ 
    // normally called @10Hz
    // read in sonar altitude
    sonar_alt = read_sonar(); //returns 0 if not healthy or if any issue.
    lrf_dist = sonar_alt;   // dist in cm. sonar_alt = rangefinder primary instance    
    if(lrf_dist==0) return false; // error

    
    // Filtrer la mesure du lidar, notamment s'il détecte les hélices ou prop-guards
    // On part du principe que le lidar est au centre du drone. On laisse une marge suppl. de 10cm
    if(lrf_dist<COPTER_DIAMETER/2+10){
        lrf_dist = 0;                   // 0 means incorrect read.
    }
    
    return true;
    
    // TO-DO
    // Voir comment gérer les mesures trop grandes en fonction des extremum de mesures du lidar
    // Voir si besoin d'ajouter un filtre passe bas
    // Voir si besoin d'ajouter un registre à décalage pour etre synchronisé entre position nacelle et distance retournée
    
}

// oa_update_map_from_lrf_read - Updates OA_Map from the new LRF read, gimbal control angles and copter position
// N'appeler cette fonction que si la lecture du lrf est correcte et à jour...
static void oa_update_map_from_lrf_read()
{   
    Vector3f update_pitch;
    float update_vector;
   
    //debug 
    /*ef_gimbal_Az = -0.6f; //2/3*pi  tester ensuite des valeurs negatives
    ef_gimbal_Ay = -0.785;//pi/4  tester ensuite 3/4*pi  et tester aussi en negatif
    ATTENTION AY PEUT ALLER DE 0 à PI ET NEGATIF AUSSI DONC CALCULS A REVOIR POUR CES CAS LA - OK VALIDE
    */
    if(real_ef_gimbal_Ay > HALF_PI){ // return angles if gimbal_Ay (vertical) is over 90° (gimbal above)
        real_ef_gimbal_Az = wrap_PI(real_ef_gimbal_Az + PI);
        real_ef_gimbal_Ay = PI - real_ef_gimbal_Ay;
    }else if(real_ef_gimbal_Ay < -HALF_PI){ // return angles if gimbal_Ay (vertical) is under -90° (gimbal below)
        real_ef_gimbal_Az = wrap_PI(real_ef_gimbal_Az + PI);
        real_ef_gimbal_Ay = -PI - real_ef_gimbal_Ay;
    }
    if((real_ef_gimbal_Az <= HALF_PI)&&(real_ef_gimbal_Az > -HALF_PI)){ // Set the vector direction from the Gimbal_Az angle
        update_pitch.x = 1.0f;     // set the sign of the x axis to 1 or -1, updated later to get this unit value on the longer x, y or z axis.
    }else{
        update_pitch.x = -1.0f;
    }
    // attention à la fonction tan qui peut donner des résultats infinis.
    update_pitch.y = update_pitch.x * constrain_float(tanf(real_ef_gimbal_Az),-1000.0f, 1000.0f); // limit tan() value because tan(PI/2) gives infinite value and that could make the other vector components inacurate once vector reduced to unity
    update_pitch.z = pythagorous2(update_pitch.x, update_pitch.y) * constrain_float(tanf(real_ef_gimbal_Ay),-1000.0f, 1000.0f);
    // the longer projection has to be =1
    update_pitch /= oa_max_vector_abs_value(update_pitch);  // division par 0 impossible
    
    /*cliSerial->printf_P(PSTR("update_pitch.x = %f, update_pitch.y = %f, update_pitch.z = %f\n"),      //debug
        update_pitch.x,
        update_pitch.y,
        update_pitch.z);*/

    int i,x,y,z,inc,i_stop;
    i_stop = (int)round(lrf_dist/(update_pitch.length()*OA_MAP_RES)); //floor() instead of round() would give better safety as, with round, we could push away an object by OA_MAP_RES/2 cm
    bool stop_update = false;
    for (i=0; !stop_update; i++){
        x = (int)floor(copter_map_index.x + update_pitch.x*i);
        y = (int)floor(copter_map_index.y + update_pitch.y*i);
        z = (int)floor(copter_map_index.z + update_pitch.z*i);
        // check if the map point to update is still in the mapping table
        stop_update = (x<0)||(x>=OA_MAP_SIZE_X)||(y<0)||(y>=OA_MAP_SIZE_Y)||(z<0)||(z>=OA_MAP_SIZE_Z);
        if(!stop_update){
            //update_vector = update_pitch.length()*(i+1)*OA_MAP_RES;
            if(i<i_stop){
                inc = -1;
            }else{
                inc = 1;
                stop_update = true;
            }
            //Values: 0=No object, 1=Not sure-to check, 2=Object detected
            oa_map[x][y][z] = constrain_int16(oa_map[x][y][z]+inc,0,2);       
        }
    }

    /* old    
    int i,x,y,z,inc;
    bool stop_update = false;
    for (i=0; !stop_update; i++){
        x = (int)floor(copter_map_index.x + update_pitch.x*i);
        y = (int)floor(copter_map_index.y + update_pitch.y*i);
        z = (int)floor(copter_map_index.z + update_pitch.z*i);
        // check if the map point to update is still in the mapping table
        stop_update = (x<0)||(x>=OA_MAP_SIZE_X)||(y<0)||(y>=OA_MAP_SIZE_Y)||(z<0)||(z>=OA_MAP_SIZE_Z);
        if(!stop_update){
            update_vector = update_pitch.length()*(i+1)*OA_MAP_RES;
            if(lrf_dist<update_vector){
                inc = 1;
                stop_update = true;
            }else{
                inc = -1;
            }
            //Values: 0=No object, 1=Not sure-to check, 2=Object detected
            oa_map[x][y][z] = constrain_int16(oa_map[x][y][z]+inc,0,2);       
        }
    }*/
    
}

// oa_check_object_in_map - Looks for any object in the map from the copter position and moving direction
static int16_t oa_check_object_in_map(float &Ay, float &Az)
{   
    bool stop_check;
    int stop_check_cnt;
    int i,j,x,y,z,m,m_sgn,n,n_sgn,iter_m,iter_n,nb_total_cells=0;
    int cell_status[OA_MAP_SIZE_X][3];  // For each check step in the map, we store the numbers of cell with values 0, 1 or 2 (that's why there are 3 columns and the maximum pitch number possible as rows)
    float cos_Ay = cosf(Ay);
    float sin_Ay = sinf(Ay);
    float cos_Az = cosf(Az);
    float sin_Az = sinf(Az);
    Vector3f Pz, Pxy, A, B, C, D;
    Vector3f check_vector_origin, orig_xy_pitch, orig_z_pitch;
    int nb_xy_cells, nb_z_cells;
    float divider;  // voir si utilisé
    float temp1, temp2;    //used to reduce the memory used
    float A_len, B_len, Pz_len, check_pitch_len, check_pitch_xy_len, Pz_case_3, Pz_case_4;
    float C_len;
    
    cliSerial->printf_P(PSTR("vel.x = %f, vel.y = %f, vel.z = %f\n"),      //debug
                                vel.x,
                                vel.y,
                                vel.z);
    cliSerial->printf_P(PSTR("Az = %f, Ay = %f, cos_Az = %f, sin_Az = %f, cos_Ay = %f, sin_Ay = %f\n"),      //debug
                                Az,
                                Ay,
                                cos_Az,
                                sin_Az,
                                cos_Ay,
                                sin_Ay);
        
    // If no velocity, just exit as we don't know where to look at. Avoid division by 0 as well.
    if(vel.length()<OA_VEL_0) return -1;
    
    
    // Init cell_status table
    for (i=0; i<OA_MAP_SIZE_X; i++){
        for (j=0; j<3; j++){
            cell_status[i][j] = 0;
        }
    }
              
    Vector3f check_pitch;
    // Resize this pitch vector, 
    check_pitch = vel/oa_max_vector_abs_value(vel);  // Voir pour intégrer ici des anticipations de mouvement (acc, commandes pilote..). Utiliser pour cela desired velocity!
                    // dans ce cas il faut recalcuiler Az et Ay par rapport à ce nouveau vecteur et non à partir de la vitesse (vel)
                    // éviter les divisions par 0. Si cette portion du code n'est pas exécutée quand vel=0, c'est ok

    /*cliSerial->printf_P(PSTR("check_pitch.x = %f, check_pitch.y = %f, check_pitch.z = %f\n"),      //debug
            check_pitch.x,
            check_pitch.y,
            check_pitch.z);*/
                    
/* OBSOLETE, corrigé en par la ligne ci-dessus
A SUPPRIMER ULTERIEUREMENT
    // A reprendre cxar ca ne prend pas en compte les valeurs négatives
    if(check_pitch.x > check_pitch.z){
        if(check_pitch.x > check_pitch.y){ // x is the longest
            i = 0;
        }else{  // y is the longest
            i = 1;
        }
    }else{
        if(check_pitch.y > check_pitch.z){ // y is the longest
            i = 1;
        }else{  // z is the longest
            i = 2;
        }
    }
    check_pitch /= check_pitch[i];
    //the longer projection HAS TO be exactelly =1 and not 0.99999
    check_pitch[i] = 1.0f;
    // that's why this is not enough: check_pitch /= max(max(check_pitch.x, check_pitch.y), check_pitch.z);
FIN DE SUPPRESSION*/

    // Il faut définir ici tous les points d'origine des vecteurs à vérifier (nombre et pos)
      /* example de code
     // Update copter map index (x/y/z index of the copter in mapping table)
        copter_map_index.x = (int)round(floor(oa_copter_pos.x/OA_MAP_RES) - oa_map_origin.x/OA_MAP_RES);
        copter_map_index.y = (int)round(floor(oa_copter_pos.y/OA_MAP_RES) - oa_map_origin.y/OA_MAP_RES);
        copter_map_index.z = (int)round(floor(oa_copter_pos.z/OA_MAP_RES) - oa_map_origin.z/OA_MAP_RES);
        */
    
    if(fabs(cos_Az) >= HALF_SQRT_2){     // means if Az € [-45;45] U [135;225]
    //if(fabs(check_pitch.x) >= fabs(check_pitch.y)){      // same condition, use the less long to compute
        //divider = cos_Az; voir si utilisé
        Pxy.x = -cos_Az*sin_Az;
        Pxy.y = cos_Az*cos_Az;
        temp1 = sin_Az;
    }else{
        //divider = sin_Az; voir si utilisé
        Pxy.x = sin_Az*sin_Az;
        Pxy.y = -cos_Az*sin_Az;
        temp1 = cos_Az;
    }
    Pxy.z = 0.0f;
    check_pitch_len = check_pitch.length();
    B = check_pitch;
    B.z = 0.0f;
    check_pitch_xy_len = B.length();
    B /= oa_max_vector_abs_value(B); // éviter les divisions par 0. Si cette portion du code n'est pas exécutée quand vel=0, c'est ok
    B_len = B.length();
    Pz_case_3 = B_len*fabs(sin_Ay);
    Pz_case_4 = B_len*fabs(cos_Ay);
    Pz_len = max(Pz_case_3, Pz_case_4);
    temp2 = Pz_len*fabs(check_pitch.z)/(check_pitch_len*B_len);
    Pz.x = temp2*B.x;
    Pz.y = temp2*B.y;
    Pz.z = -sgn(check_pitch.z)*check_pitch_xy_len*Pz_len/check_pitch_len;
    
    //définis sur excel, voir si utilisé plus tard
    // utilisé 1x    Pxy.length();   //||Pxy||
    // utilisé 1x    B.length();   //||B||
    nb_xy_cells = (int)ceil((W_proj/OA_MAP_RES)/Pxy.length()); 
    nb_z_cells = (int)ceil((S_proj/OA_MAP_RES)/Pz_len);
    
    //debug
    //nb_xy_cells = 1; //debug
    //nb_z_cells = 1; //debug
    
    /*cliSerial->printf_P(PSTR("nb_xy_cells = %d, nb_z_cells = %d\n"),      //debug
        nb_xy_cells,
        nb_z_cells);*/
    
    // define check_vectors origins      
    for (iter_m=0; iter_m<=nb_xy_cells; iter_m++){  // xy path cells check
        m = iter_m-nb_xy_cells/2;
        m_sgn = sgn(m);                     // sign of m = 1 or -1
        A_len = fabs(fmod(m*temp1, B_len)); // fabs() is used in case of fmod would return negative values (excel doesn't during simulations). Normally we should do fabs(m*temp1), excel needs it but here it seems to be ok without
        A = B*A_len/B_len;                  // évite de détailler les 3 composantes du vecteur
        orig_xy_pitch = Pxy*m+A*m_sgn;     
        if(A_len>=(B_len/2)){
            orig_xy_pitch -= B*m_sgn;
        }
        /*cliSerial->printf_P(PSTR("A_len = %f, B_len = %f, m(xy)= %d, orig_xy_pitch.x = %f, orig_xy_pitch.y = %f, orig_xy_pitch.z = %f\n"),      //debug
        A_len,
        B_len,
        m,
        orig_xy_pitch.x,
        orig_xy_pitch.y,
        orig_xy_pitch.z);*/

        for (iter_n=0; iter_n<=nb_z_cells; iter_n++){  // z path cells check
            n = iter_n-nb_z_cells/2;
            n_sgn = sgn(n);
            C_len = fabs(fmod(n*min(Pz_case_3, Pz_case_4), check_pitch_len));   // fabs() is used in case of fmod would return negative values. Normally we should do abs(n), excel needs it but here it seems to be ok without
            C = check_pitch*C_len/check_pitch_len;
            if(Pz_case_3>Pz_case_4){
                orig_z_pitch = Pz*n+C*n_sgn;
                if(C_len>check_pitch_len/2) orig_z_pitch -= check_pitch*n_sgn;
            }else{
                orig_z_pitch = Pz*n-C*n_sgn;
                if(C_len>check_pitch_len/2) orig_z_pitch += check_pitch*n_sgn;
            }
            
            /*cliSerial->printf_P(PSTR("n(z)= %d, orig_z_pitch.x = %f, orig_z_pitch.y = %f, orig_z_pitch.z = %f\n"),      //debug
            n,
            orig_z_pitch.x,
            orig_z_pitch.y,
            orig_z_pitch.z);*/
            
            check_vector_origin = copter_map_index + orig_xy_pitch + orig_z_pitch;
            
            /*cliSerial->printf_P(PSTR("check_vector_origin.x = %f, check_vector_origin.y = %f, check_vector_origin.z = %f\n"),      //debug
                    check_vector_origin.x,
                    check_vector_origin.y,
                    check_vector_origin.z);*/
    
            //pour chacun de ces vecteurs, procéder à la vérif
            stop_check_cnt = 0;
            for (i=0; stop_check_cnt<3; i++){    // ne pas arreter au premier stop_check car on pourrait avoir une origine de vecteur en-dehors du mapping mais ses pas suivants dans le mapping
                x = (int)floor(check_vector_origin.x + check_pitch.x*i);
                y = (int)floor(check_vector_origin.y + check_pitch.y*i);
                z = (int)floor(check_vector_origin.z + check_pitch.z*i);
                // ensure the map point to check is still in the mapping table
                stop_check = (x<0)||(x>=OA_MAP_SIZE_X)||(y<0)||(y>=OA_MAP_SIZE_Y)||(z<0)||(z>=OA_MAP_SIZE_Z);
                if(!stop_check){
                    j = oa_map[x][y][z];
                    cell_status[i][j] += 1;  
                }else{
                    stop_check_cnt++;
                }
            }
        }
    }
    
    // Analyse cell_status table and find out the minimal safe distance
    // This distance will be used to limit copter position and velocity and also to focus the scan algo on the right window (wider scan beam with shortest distance)
    stop_check = false;
    for (i=0; (i<OA_MAP_SIZE_X) && !stop_check; i++){
        //Values: 0=No object, 1=Not sure-to check, 2=Object detected
        nb_total_cells = cell_status[i][0]+cell_status[i][1]+cell_status[i][2];
        if((nb_total_cells>0) && (cell_status[i][2]==0)){   // there are checked cells and no object for this step. Let's check now the ratio of safe cells
                                                            // to-do : check if we could allow a couple of objects or not because it will be hard to clear a fake object detection in the mapping table due to low scan rate...
            object_detected = false;
            //required to be safe: cell_status[i][0] / nb_total_cells >= OA_CHECK_SAFE_RATIO
            if(cell_status[i][0] < nb_total_cells*OA_CHECK_SAFE_RATIO){
                stop_check = true;
            }
        }else{
            stop_check = true;
            if(cell_status[i][2]>0) object_detected = true;
        }
        
        cliSerial->printf_P(PSTR("Cell_Status: i = %d, cell_status[i][0] = %d, cell_status[i][1] = %d, cell_status[i][2] = %d\n"),      //debug
        i,
        cell_status[i][0],
        cell_status[i][1],
        cell_status[i][2]);
        
        
    } 
    return (int)(check_pitch_len*(i-1)*OA_MAP_RES);  // cm  
}

// oa_select_scan_algo - Returns the scan algo to run from the copter current state (vel and y_angle=Tilt_angle)
static int16_t oa_select_scan_algo(float &Angle_y)
{   
    switch (gimbal_type){
        case PAN_TILT_2_AXIS:
            if(!oa_use_map){
                return NO_MAP_SCAN;
            }
        
            if(vel.length()<OA_VEL_0){
                return GENERAL_SCAN_NO_VEL;
            }
            // compute Ay_Max = max angle to define if the scan is mainly horizontal or vertical
            float Ay_max, delta_Y;
            delta_Y = asinf(S_proj/(2*OA_MIN_DIST));
            /*
            cliSerial->printf_P(PSTR("Ay_max_plus_y_delta = %f, delta_Y = %f\n"),      //debug
            Ay_max_plus_y_delta,
            delta_Y);                             //debug
            */
            switch(gimbal_position){
                case ABOVE:
                    // check if the gimbal is able to scan the relevant (destination) area
                    if(Angle_y>=uav_y_reduced_inclination){
                        Ay_max = Ay_max_plus_y_delta - delta_Y;
                        if(Angle_y-uav_y_reduced_inclination>Ay_max){
                            return SCANNABLE_VERTICAL;
                        }else{
                            return SCANNABLE_HORIZONTAL;
                        }
                    }else if(vel_xy>OA_VEL_0){
                        return UNSCANNABLE_Z_ANTICIPATE;
                    }else{
                        return GENERAL_SCAN_NO_VEL; // if vel is mainly vertical & negative, we cannot anticipate so we'll make a general scan just like if there was no velocity at all
                    }
                    break;
                case BELOW:
                    // check if the gimbal is able to scan the relevant (destination) area
                    if(Angle_y<=uav_y_reduced_inclination){
                        Ay_max = Ay_max_plus_y_delta + delta_Y;
                        if(Angle_y-uav_y_reduced_inclination<Ay_max){
                            return SCANNABLE_VERTICAL;
                        }else{
                            return SCANNABLE_HORIZONTAL;
                        }
                    }else if(vel_xy>OA_VEL_0){
                        return UNSCANNABLE_Z_ANTICIPATE;
                    }else{
                        return GENERAL_SCAN_NO_VEL; // if vel is mainly vertical & negative, we cannot anticipate so we'll make a general scan just like if there was no velocity at all
                    }
                    break;
            }
            break;
    }
    // to keep compiler happy to-do : a suppr car certainement inutile au final
    return GENERAL_SCAN_NO_VEL;
}

// oa_run_scan_algo - Run the selected scan algorithm
static void oa_run_scan_algo(int16_t &scan_algo, float &Ay, float &Az)
{
    float ef_scan_pitch;
    switch (gimbal_type){
        case PAN_TILT_2_AXIS:
            switch (scan_algo){
                case NO_MAP_SCAN:               // no GPS, just aim in the moving direction from roll/pitch/yaw angles
                                                // to-do  voir pour un management basé sur capteurs virtuels. mini mapping angulaire à voir
                    //tbd
                    // voir ef ou bf, car il faudra peut etre identifier 2 cas sans mapping. si on avance on vise dans la direction. sinon on scanne tout autour dans un mini buffer (capteurs virtuels)
                            // si laissé ici, évaluer Az (difficile, mieux vaut des capteurs virtuels)
                    
                    //bf_angles used
                    use_ef_or_bf_gimbal_angles = BF_ANGLES;
                    break;
                    
                case GENERAL_SCAN_NO_VEL:       // when copter is loitering with no velocity, we scan everywhere because we don't know in which direction will be the next command.
                                                // a periodic scan of the top/bottom/front/... depending on the gimbal position is compulsory
                    //bf_angles used
                    use_ef_or_bf_gimbal_angles = BF_ANGLES;
                    
                    if(scan_algo != last_scan_algo){ // Init scan_algo
                        bf_gimbal_Ay = 0;
                        bf_gimbal_Az = -PI;
                        scan_step = 0;
                        last_scan_algo = scan_algo;
                        if(gimbal_position == ABOVE){
                            y_sign = 1;
                        }else{
                            y_sign = -1;
                        }
                        return; // means apply those first controls
                    }
                    switch (scan_step){
                        case 0:
                            bf_gimbal_Az += DEFAULT_SCAN_PITCH;
                            if(bf_gimbal_Az>PI-DEFAULT_SCAN_PITCH) scan_step++;      
                            break;
                        case 1:
                            bf_gimbal_Ay += y_sign * DEFAULT_SCAN_PITCH;
                            if(fabs(bf_gimbal_Ay)>HALF_PI) bf_gimbal_Ay = 0;
                            scan_step++;
                            break;
                        case 2:
                            bf_gimbal_Az -= DEFAULT_SCAN_PITCH;
                            if(bf_gimbal_Az<-PI+DEFAULT_SCAN_PITCH) scan_step++;      
                            break;
                        case 3:
                            bf_gimbal_Ay += y_sign * DEFAULT_SCAN_PITCH;    // no need to check here if gimbal_Ay is over +/- HALF_PI as it can reach up to +/-PI angles
                            scan_step = 0;
                            break;                      
                    }
                    //ef_gimbal_Az = wrap_PI(ef_gimbal_Az+DEFAULT_SCAN_PITCH);
                    
                    break;
                    
                case SCANNABLE_VERTICAL:        // copter moving, the gimbal needs to scan mainly what's up/down (vertically)
                    // bf angles de préférence
                    use_ef_or_bf_gimbal_angles = BF_ANGLES;
                    
                    if(scan_algo != last_scan_algo){ // Init scan_algo
                        bf_gimbal_Ay = y_sign * QUARTER_PI;
                        bf_gimbal_Az = -HALF_PI;
                        scan_step = 0;
                        gimbal_Az_scan_offset = 0;
                        last_scan_algo = scan_algo;
                        inc_offset = true;
                        if(gimbal_position == ABOVE){
                            y_sign = 1;
                        }else{
                            y_sign = -1;
                        }
                        return; // means apply those first controls
                    }
                    switch (scan_step){
                        case 0:
                            bf_gimbal_Ay += y_sign * DEFAULT_SCAN_PITCH;
                            if(fabs(bf_gimbal_Ay)>HALF_PI+QUARTER_PI) scan_step++;
                            break;
                        case 1:
                        case 3:
                            if(bf_gimbal_Az<=-HALF_PI){
                                z_sign = 1;
                                bf_gimbal_Az = -HALF_PI;
                                inc_offset = true;
                            }else if(bf_gimbal_Az>=HALF_PI){   
                                z_sign = -1;
                                bf_gimbal_Az = HALF_PI;
                                inc_offset = true;
                            }
                            if(inc_offset){     // 1 cycle finished, apply Az offset to avoid scanning always the same things
                                gimbal_Az_scan_offset += DEFAULT_SCAN_PITCH;
                                if(gimbal_Az_scan_offset>=QUARTER_PI)gimbal_Az_scan_offset = 0;
                                bf_gimbal_Az += gimbal_Az_scan_offset;
                                inc_offset = false;
                            }
                            bf_gimbal_Az += z_sign*QUARTER_PI;
                            if(scan_step == 3){     
                                scan_step = 0;
                            }else{
                                scan_step++;
                            }
                            break;
                        case 2:
                            bf_gimbal_Ay -= y_sign * DEFAULT_SCAN_PITCH;
                            if(fabs(bf_gimbal_Ay)<QUARTER_PI) scan_step++;
                            break;                  
                    }
                    
                    break;
                    
                case SCANNABLE_HORIZONTAL:      // copter moving, the gimbal needs to scan mainly what's around (horizontally)
                case UNSCANNABLE_Z_ANTICIPATE:  // copter moving, gimbal unable to scan the relevant path. But we can still check in case of the vel.z changes 
                                // (eg: gimbal above, copter moving forward down, we'll scan in the moving direction (forward) in case of the vel.z becomes positive the "new path" will be already scanned. We'll keep an eye as well on the roof (top)

                    //use ef_gimbal angles
                    use_ef_or_bf_gimbal_angles = EF_ANGLES;
                    
                    if(scan_algo != last_scan_algo){ // Init scan_algo
                        des_ef_gimbal_Ay = Ay;
                        des_ef_gimbal_Az = Az;
                        scan_step = 0;
                        gimbal_Az_scan_offset = 0;
                        gimbal_Ay_scan_offset = 0;
                        y_sign = -1;
                        z_sign = 1;
                        last_scan_algo = scan_algo;
                        pitch_count = (int8_t)ceil(W_proj/(2*OA_SCAN_RES));   // init pitch count for step0 => Az axis
                        //inc_offset = true;
                        return; // means apply those first controls
                    }
                    // compute Ay and Az pitches from the current window to scan. 
                    //float gimbal_Ay_pitch = xxx;
                    //float gimbal_Az_pitch = xxx;
                    float scan_dist, angle_pitch;
                    
                    scan_dist = max((float)safe_dist, stopping_dist);
                    scan_dist += 2*OA_MAP_RES;  // give some head_room, this could be increased if needed
                    angle_pitch = fast_atan2(OA_SCAN_RES, scan_dist);    // same angle pitch for Ay and Az
                    
                    if(scan_algo == UNSCANNABLE_Z_ANTICIPATE){      // Force a pan scan only around the moving direction. gimbal_Ay_scan_offset is kept = 0 (init value) to scan horizontally.
                        scan_step = 0;    
                        pitch_count = (int8_t)ceil(W_proj/(2*OA_SCAN_RES));
                    }
                    
                    switch (scan_step){
                        case 0:
                        case 2:
                        case 4:
                        case 6:
                            gimbal_Az_scan_offset += z_sign;
                            if(fabs(gimbal_Az_scan_offset) >= pitch_count){
                                if(z_sign == 1){     
                                    z_sign = -1;
                                }else{
                                    z_sign = 1;
                                    //scan_step = 0;
                                }
                                // compute the pitch count for next step (here, Ay axis)
                                pitch_count = (int8_t)ceil(S_proj/(2*OA_SCAN_RES));
                                scan_step++;
                            }
                            break;
                        case 1:
                        case 5:
                            gimbal_Ay_scan_offset += y_sign;
                            if(fabs(gimbal_Ay_scan_offset) >= pitch_count){
                                if(y_sign == 1){     
                                    y_sign = -1;
                                }else{
                                    y_sign = 1;
                                }
                                // compute the pitch count for next step (here, Az axis)
                                pitch_count = (int8_t)ceil(W_proj/(2*OA_SCAN_RES));
                                scan_step++;
                            }
                            break;   
                        case 3:
                            gimbal_Ay_scan_offset += y_sign;
                            if(gimbal_Ay_scan_offset>=0){
                                pitch_count = (int8_t)ceil(W_proj/(2*OA_SCAN_RES));
                                scan_step++;
                            }
                            break;
                        case 7:
                            gimbal_Ay_scan_offset += y_sign;
                            if(gimbal_Ay_scan_offset<=0){
                                pitch_count = (int8_t)ceil(W_proj/(2*OA_SCAN_RES));
                                scan_step = 0;
                            }
                            break;
                    }          

                    des_ef_gimbal_Ay = Ay + gimbal_Ay_scan_offset * angle_pitch;
                    des_ef_gimbal_Az = Az + gimbal_Az_scan_offset * angle_pitch;
                    
                    /*
                    cliSerial->printf_P(PSTR("scan_dist = %f, stopping_dist = %f, pitch_count = %d\n"),      //debug
                    scan_dist,
                    stopping_dist,
                    pitch_count);                             //debug
                    */
                    break;
            }
            
            // voir s'il faut contraindre les angles bf et ef dans leurs plages d'utilisation normale ici (surtout les angles BF)
            // ok c'est déjà contraint plus haut
            break;
    }
}


// oa_gimbal_control - Controls the oa gimbal (send target angles to the mount) and correct the real_targeted angles for the future map update
static void oa_gimbal_control(int16_t &scan_algo, float &Az)
{
    float gimbal_y_reduced_inclination;         // in rad. UAV inclination on gimbal_Az aiming direction, reduced to Y axis
    // Voir pour placer ce qui suit dans une fonction appelée par l'algo de scan. on actualise uniquement si le gimbal_control_lock est à 0
    //Délimiter la zone de balayage et vérifier que les butées de la nacelle n'imposent pas des rotation totales
    // sur le pan de la nacelle. A la limite utiliser le cas de plus défavorable (vitesse nulle - fenetre de scan de 120°)
    // et définir si l'angle du tilt doit être inversé ou non
    // Gimbal reverse check - ONLY FOR SCAN CASE N°3 = Horizontal vel mainly (revient au cas où on utilise des angles ef)

        /*
        //simulate gimbal angles debug
        //0-1000 -> -pi+pi scaling
        des_ef_gimbal_Ay = (float)(g.rc_7.control_in-500)*0.006283f; //debug
        des_ef_gimbal_Az = (float)(g.rc_8.control_in-500)*0.006283f; //debug
        */
    
    // to-do: vérifier calculs depuis l'ajout de l'offset
    
    // if ef_gimbal angles are used by the current scan algo, convert them to bf angles
    if(use_ef_or_bf_gimbal_angles == EF_ANGLES){
        // for these scan algo, use ef_gimbal angles and transpose them to bf angles
        //to-do: voir sir les 3 calculs suivants sont exécutés un iquement lors de l'utilisation du mapping, car autrement autant raisonner direct par rapport au roll/pitch?...
        gimbal_y_reduced_inclination = ahrs.pitch*cosf(wrap_PI(ahrs.yaw+GIMBAL_YAW_OFFSET-des_ef_gimbal_Az)) + ahrs.roll*sinf(wrap_PI(ahrs.yaw+GIMBAL_YAW_OFFSET-des_ef_gimbal_Az));     // in rad. Gimbal inclination reduced to Y axis
        //bf_gimbal_Ay = wrap_PI(des_ef_gimbal_Ay-gimbal_y_reduced_inclination); //ancien testé OK mais pas complètement à mon avis
        bf_gimbal_Ay = des_ef_gimbal_Ay-gimbal_y_reduced_inclination;   // will be constrained later
        bf_gimbal_Az = wrap_PI(des_ef_gimbal_Az-(ahrs.yaw+GIMBAL_YAW_OFFSET));        
        
        // reverse gimbal only for these scan algorithms
        // to-do : spécifique à la nacelle 2 axes, voir pour ajouter un switch case ici
        //DEBUT DU SWITCH CASE TYPE NACELLE - ON DEVELOPPE ICI LA NACELLE SPE
        
        if(!reverse_gimbal){
            // Reverse gimbal if scanning around the -180/+180 BF pan limit angles with a decent margin
            // ce n'est pas la direction de gimbal_az qu'il faut vérifier mais bien Az directement!
            //if((Az < wrap_PI(ahrs.yaw + GIMBAL_YAW_OFFSET - PI + MAX_SCAN_PAN_ANGLE/2)) && (Az > wrap_PI(ahrs.yaw + GIMBAL_YAW_OFFSET + PI - MAX_SCAN_PAN_ANGLE/2))){
            float offset_angle = fabs(Az-(ahrs.yaw+GIMBAL_YAW_OFFSET));
            if((offset_angle > PI-MAX_SCAN_PAN_ANGLE/2)&&(offset_angle < PI+MAX_SCAN_PAN_ANGLE/2)){
                reverse_gimbal = true;
            }
        }else{  // Gimbal reversed
            // Stop reversing gimbal if scanning around the 0° BF pan angle with a decent margin - that gives a +/- 120° hysteresis to the reverse_gimbal condition
            //if((Az > wrap_PI(ahrs.yaw + GIMBAL_YAW_OFFSET - MAX_SCAN_PAN_ANGLE/2)) && (Az < wrap_PI(ahrs.yaw + GIMBAL_YAW_OFFSET + MAX_SCAN_PAN_ANGLE/2))){
            if(fabs(wrap_PI(Az-(ahrs.yaw+GIMBAL_YAW_OFFSET)))<MAX_SCAN_PAN_ANGLE/2){
                reverse_gimbal = false;
            }
        }
        
        // check if desired ef_gimbal angles could be mechanically reached by gimbal. If not, correct them
        // + constraint bf_gimbal angles
        real_ef_gimbal_Ay = des_ef_gimbal_Ay;   // by default, real = des
        switch(gimbal_position){
            case ABOVE:
                if(bf_gimbal_Ay<0.0f){ 
                    real_ef_gimbal_Ay = gimbal_y_reduced_inclination;       // gimbal_y_reduced_inclination is positive here
                }else if(bf_gimbal_Ay>PI){
                    real_ef_gimbal_Ay = PI + gimbal_y_reduced_inclination;  // gimbal_y_reduced_inclination is negative here
                }
                constrain_float(bf_gimbal_Ay,0.0f, PI);
                break;
            case BELOW:
                if(bf_gimbal_Ay>0.0f){ 
                    real_ef_gimbal_Ay = gimbal_y_reduced_inclination;       // gimbal_y_reduced_inclination is negative here
                }else if(bf_gimbal_Ay<-PI){
                    real_ef_gimbal_Ay = -PI + gimbal_y_reduced_inclination; // gimbal_y_reduced_inclination is positive here
                }
                constrain_float(bf_gimbal_Ay,-PI, 0.0f);
                break;
        }
        real_ef_gimbal_Az = des_ef_gimbal_Az;   //always true for this gimbal type
        // to-do : vérifier ces calculs
        
        // only then, compute bf angles for reversed gimbal
        if(reverse_gimbal){     
            bf_gimbal_Az = wrap_PI(bf_gimbal_Az + PI);
            switch(gimbal_position){
                case ABOVE:
                    bf_gimbal_Ay = PI - bf_gimbal_Ay; // no need to constraint between [0;pi] from tests
                    break;
                case BELOW:
                    bf_gimbal_Ay = -PI - bf_gimbal_Ay;
                    break;
            }
        }
    }else{  // If BF_angles are used
        //reverse_gimbal = false; // a suppr car inutile suivant où est gérée le revert
        // les bf_angles sont utilisés tels quels
        // compute real_ef_gimbal angles from bf_gimbal angles (will be used to update
        // voir pour mutualiser le bornage des bf angles et le calcul des real_ef angles ustilisé plus haut dans le cas de EF
        real_ef_gimbal_Az = wrap_PI(bf_gimbal_Az + ahrs.yaw + GIMBAL_YAW_OFFSET);
        gimbal_y_reduced_inclination = ahrs.pitch*cosf(wrap_PI(ahrs.yaw+GIMBAL_YAW_OFFSET-real_ef_gimbal_Az)) + ahrs.roll*sinf(wrap_PI(ahrs.yaw+GIMBAL_YAW_OFFSET-real_ef_gimbal_Az));     // in rad. Gimbal inclination reduced to Y axis
        real_ef_gimbal_Ay = wrap_PI(bf_gimbal_Ay + gimbal_y_reduced_inclination);
    }

    
    // Check if gimbal is in the right mode and update it
    if(camera_mount.get_mode() != MAV_MOUNT_MODE_OA_BYPASS){
        camera_mount.set_mode(MAV_MOUNT_MODE_OA_BYPASS);	
    }    
    camera_mount.set_oa_control_angles(0.0f, degrees(bf_gimbal_Ay), degrees(bf_gimbal_Az)); // roll, tilt, pan angles in degrees
    camera_mount.update_mount_position();
    
    
    /*
    // debug
    cliSerial->printf_P(PSTR("des_ef_gimbal_Ay = %f, des_ef_gimbal_Az = %f, real_ef_gimbal_Ay = %f, real_ef_gimbal_Az = %f, bf_gimbal_Ay = %f, bf_gimbal_Az = %f, gimbal_y_reduced_inclination = %f, reverse_gimbal = %d\n"),      //debug
                        des_ef_gimbal_Ay,
                        des_ef_gimbal_Az,
                        real_ef_gimbal_Ay,
                        real_ef_gimbal_Az,
                        bf_gimbal_Ay,
                        bf_gimbal_Az,
                        gimbal_y_reduced_inclination,
                        reverse_gimbal);    //debug
    */
    
}

// oa_max_vector_abs_value - Returns the absolute longest component value of the passed vector.
static float oa_max_vector_abs_value(const Vector3f& v)
{
    return max(max(fabs(v.x), fabs(v.y)), fabs(v.z));
}

// oa_set_vel_xy - called by flight modes to update vel vector with anticipated values (from pilot inputs)
static void oa_set_vel_xyz(Vector2f anticipated_pilot_xy_des_vel, float target_climb_rate)
{
    vel.x = anticipated_pilot_xy_des_vel.x;
    vel.y = anticipated_pilot_xy_des_vel.y;
    vel.z = target_climb_rate;
}

// oa_get_max_braking_dist - returns the maximum braking distance for the copter. calculated from anticipated heading(moving direction)
static int16_t oa_get_max_braking_dist()
{
    return safe_dist-OA_MIN_DIST;   //can be negative
}

// oa_enabled - returns true is OA is running
static bool oa_enabled()
{
    //to-do : check here if everything works fine with OA and return its status
    //eg : if lrf does'nt work, or maybe a gimbal status depending on gimbals...
    return true;
}

// oa_is_object_detected - returns true if an object is detected or not. usefull to know if the safe distance should be used for vel limitation or stopping point
static bool oa_is_object_detected()
{
    return object_detected;
}

#endif  // OA_ENABLED == ENABLED



    //limiter angles Ay sur des valeurs positives
    // Adapter l'algo de scan au différentes phases
    // 1 : Vitesse principalement verticale positive - scan du haut
    // 2 : Vitesse principalement verticale négative ou globalement nulle - on scan à 360° pour identifier les éventuels objets latéraux pouvant déborder dessous le drone
    // 3 : Vitesse principalement horizontale - on scan devant nous




    //exemples start
    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    //  roll_angle = (float)fast_atan((-poshold.wind_comp_ef.x*ahrs.sin_yaw() + poshold.wind_comp_ef.y*ahrs.cos_yaw())/981)*(18000/M_PI);
    //  pitch_angle = (float)fast_atan(-(poshold.wind_comp_ef.x*ahrs.cos_yaw() + poshold.wind_comp_ef.y*ahrs.sin_yaw())/981)*(18000/M_PI);
    // void AP_Mount::set_control_angles(float roll, float tilt, float pan)
 

    // save du general scan no vel, simplifié pour commencer, à améliorer plus tard au besoin
    /*switch (scan_step){
                        case 0:
                            bf_gimbal_Az += DEFAULT_SCAN_PITCH;
                            if(bf_gimbal_Az>HALF_PI) scan_step++;      
                            break;
                        case 1:
                        case 4:
                            bf_gimbal_Ay = y_sign * HALF_PI;
                            scan_step++;
                        case 2:
                            bf_gimbal_Ay = y_sign * PI;
                            scan_step++;
                        case 3:
                            bf_gimbal_Az -= DEFAULT_SCAN_PITCH;
                            if(bf_gimbal_Az<-HALF_PI) scan_step++;
                            //bf_gimbal_Az = HALF_PI;
                            // AY A 90° PUIS ENSUITE AY A 180 ET DECROITRE AZ; VOIR POUR LES PITCH AZ DE FACON A NE PAS SCANNER PLUSIEURS FOIS LES MEME ANGLES     
                            break;                        
                    }
                    */
                    
                    
                    
                    
                                        
                    
         /*           
        if(!reverse_gimbal){
            // Reverse gimbal if scanning around the -180/+180 BF pan limit angles with a decent margin
            if((ef_gimbal_Az < wrap_180_cd_float(ahrs.yaw + GIMBAL_YAW_OFFSET - PI + MAX_SCAN_PAN_ANGLE/2)) || (ef_gimbal_Az > wrap_180_cd_float(ahrs.yaw + GIMBAL_YAW_OFFSET + PI - MAX_SCAN_PAN_ANGLE/2))){
                reverse_gimbal = true;
            }
        }else{  // Gimbal reversed
            // Stop reversing gimbal if scanning around the 0° BF pan angle with a decent margin - that gives a +/- 120° hysteresis to the reverse_gimbal condition
            if((ef_gimbal_Az > wrap_180_cd_float(ahrs.yaw + GIMBAL_YAW_OFFSET - MAX_SCAN_PAN_ANGLE/2)) && (ef_gimbal_Az < wrap_180_cd_float(ahrs.yaw + GIMBAL_YAW_OFFSET + MAX_SCAN_PAN_ANGLE/2))){
                reverse_gimbal = false;
            }
        }*/
                    
                    
                    /*
                    case SCANNABLE_HORIZONTAL:      // copter moving, the gimbal needs to scan mainly what's around (horizontally)
                case UNSCANNABLE_Z_ANTICIPATE:  // copter moving, gimbal unable to scan the relevant path. But we can still check in case of the vel.z changes 
                                // (eg: gimbal above, copter moving forward down, we'll scan in the moving direction (forward) in case of the vel.z becomes positive the "new path" will be already scanned. We'll keep an eye as well on the roof (top)

                    //use ef_gimbal angles
                    use_ef_or_bf_gimbal_angles = EF_ANGLES;
                    
                    if(scan_algo != last_scan_algo){ // Init scan_algo
                        des_ef_gimbal_Ay = Ay;
                        des_ef_gimbal_Az = Az;
                        scan_step = 0;
                        gimbal_Az_scan_offset = 0;
                        gimbal_Ay_scan_offset = 0;
                        y_sign = -1;
                        z_sign = 1;
                        last_scan_algo = scan_algo;
                        //inc_offset = true;
                        return; // means apply those first controls
                    }
                    // compute Ay and Az pitches from the current window to scan. 
                    //float gimbal_Ay_pitch = xxx;
                    //float gimbal_Az_pitch = xxx;
                    float scan_dist, angle_pitch;
                    
                    scan_dist = max((float)safe_dist, stopping_dist);
                    scan_dist += 2*OA_MAP_RES;  // give some head_room, this could be increased if needed
                    angle_pitch = fast_atan2(OA_SCAN_RES, scan_dist);    // same angle pitch for Ay and Az
                    
                    if(scan_algo == UNSCANNABLE_Z_ANTICIPATE) scan_step = 0;    // Force a pan scan only around the moving direction. gimbal_Ay_scan_offset is kept = 0 (init value) to scan horizontally.
                    
                    switch (scan_step){
                        case 0:
                        case 2:
                        case 4:
                        case 6:
                            gimbal_Az_scan_offset += z_sign * angle_pitch;
                            if(fabs(gimbal_Az_scan_offset)/angle_pitch >= W_proj/(2*OA_SCAN_RES)){
                                if(z_sign == 1){     
                                    z_sign = -1;
                                }else{
                                    z_sign = 1;
                                    //scan_step = 0;
                                }
                                scan_step++;
                            }
                            break;
                        case 1:
                        case 5:
                            gimbal_Ay_scan_offset += y_sign * angle_pitch;
                            if(fabs(gimbal_Ay_scan_offset)/angle_pitch >= S_proj/(2*OA_SCAN_RES)){
                                if(y_sign == 1){     
                                    y_sign = -1;
                                }else{
                                    y_sign = 1;
                                }
                                scan_step++;
                            }
                            break;   
                        case 3:
                            gimbal_Ay_scan_offset += y_sign * angle_pitch;
                            if(gimbal_Ay_scan_offset>0) scan_step++;
                            break;
                        case 7:
                            gimbal_Ay_scan_offset += y_sign * angle_pitch;
                            if(gimbal_Ay_scan_offset<0) scan_step = 0;
                            break;
                    }          

                    des_ef_gimbal_Ay = Ay + gimbal_Ay_scan_offset;
                    des_ef_gimbal_Az = Az + gimbal_Az_scan_offset;
                    
                    
                    cliSerial->printf_P(PSTR("scan_dist = %f, stopping_dist = %f\n"),      //debug
                    scan_dist,
                    stopping_dist);                             //debug
                    
                    break;*/
