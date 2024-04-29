/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATT 20
#define PRIORITY_TWD 29



/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_battery, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_wd, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_mutex_create(&mutex_cam, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_mutex_create(&mutex_comRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camOpen, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_askArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_posCheck, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reload, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCam, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    
    }
    if (err = rt_task_create(&th_reload, "th_reload", 0, PRIORITY_TWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	
	if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	

	
  
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // get the state of the battery and display it
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Reload of the watchdog
    if (err = rt_task_start(&th_reload, (void(*)(void*)) & Tasks::ReloadTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    //Camera, Arena, Position of robots 
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::CameraTask, this)) {
    cerr << "Error task start: " << strerror(-err) << endl << flush;
    exit(EXIT_FAILURE);
    }



    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

// Our tasks ----------------------------------------------------------------------------


/**
 * @brief Thread handling the reaload of the watchdog
 */
void Tasks::ReloadTask(void *arg) {
    Message * msgSend;
    cout << "Start Reload Task " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_reload, TM_INFINITE);             //The task is blocked if the Watchdog checkbox is not checked
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000); //Do the task every 1s
    while (1) {
        rt_task_wait_period(NULL);
	    
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.ReloadWD());     //Send a message to the Robot to make the Robot reset the timer of the watchdog
        rt_mutex_release(&mutex_robot);
    }
}

/**
 * @brief Thread : Open and Close the camera, get images of the camera, draw arena and robots, get positions of robot
 */
void Tasks::CameraTask(void *arg) {
    int status;
    int com_err;
   MessageImg * msgSend;
   camera = new Camera();
   MessagePosition * msgPos;

    
    
    cout << "Start Camera Task " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeComRobot starts here                                                  */
    /**************************************************************************************/
     rt_task_set_periodic(NULL, TM_NOW, 200000000); //100ms
    Arena arena;

    while (1) {
        
        cout << "Wait for open camera" << __PRETTY_FUNCTION__ << endl << flush;
        rt_sem_p(&sem_openCam, TM_INFINITE);
        if (CamOpen) {
            rt_mutex_acquire(&mutex_cam, TM_INFINITE);       
            camera->Open();
            rt_mutex_release(&mutex_cam);
            while(CamOpen) {
                while(!AskArena && CamOpen) {
                        rt_task_wait_period(NULL);
                    rt_mutex_acquire(&mutex_cam, TM_INFINITE);       
                    Img img = camera->Grab();
                    //Draw Arena
                    if (draw) {
                            img.DrawArena(arena);
                            if (posCheck) {
                                //Draw robot
                                list<Position> pos = img.SearchRobot(arena);
                                
                                if (!pos.empty()) {
                                    
                                    for (Position p : pos) {
                                        img.DrawRobot(p);
                                        msgPos = new MessagePosition(MESSAGE_CAM_POSITION, p);
                                        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                                        monitor.Write(msgPos); // The message is deleted with the Write
                                        rt_mutex_release(&mutex_monitor);
                                    }
                                }
                                else {
                                    cout << "Robot not found : Position (-1,-1)" << __PRETTY_FUNCTION__ << endl << flush;
                                }

                            }
                        }
                   

                    //cout << "img taille : " << img.ToString() << endl << flush;
                    
                    msgSend = new MessageImg(MESSAGE_CAM_IMAGE,&img);
                        
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(msgSend); // The message is deleted with the Write
                    rt_mutex_release(&mutex_monitor);
                    rt_mutex_release(&mutex_cam);

                  
                
                }
                if (AskArena) {
                    
                    Img last_image = camera->Grab();
                       
                             arena=last_image.SearchArena();

                           
                            if (draw) {
                                last_image.DrawArena(arena);
                                SavedArena=last_image.Copy();
                                 msgSend = new MessageImg(MESSAGE_CAM_IMAGE,&last_image);
                                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                                monitor.Write(msgSend); // The message is deleted with the Write
                                rt_mutex_release(&mutex_monitor);
                                rt_mutex_release(&mutex_cam);
                            }
                             rt_sem_p(&sem_arena, TM_INFINITE);

                            
                       
                    rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
                    AskArena=false;
                    rt_mutex_release(&mutex_askArena);

                }
            }
        }
        cout << "Close Camera" << __PRETTY_FUNCTION__ << endl << flush;

        rt_mutex_acquire(&mutex_cam, TM_INFINITE);       
        camera->Close();
        rt_mutex_release(&mutex_cam);

             
    }
     
}



/**
 * @brief Thread handling control of the battery
 */
void Tasks::GetBatteryTask(void *arg) {
    int rs;
    Message* batt_level;
    
    cout << "Start getBattery Task " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000); //500ms

    while (1) {
        
        //get battery from ComRobot message
        
        rt_task_wait_period(NULL);

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);

        if (rs == 1 && getBattery==1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            batt_level = robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);

            
            //send to monitor
            WriteInQueue(&q_messageToMon, batt_level);

            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            getBattery = 0;
            rt_mutex_release(&mutex_battery);
        }
    }
}


/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    int watchdog;       //1 is watchdog is activated, 0 if not
    Message * msgSend;

    cout << "Start Robot Task " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
	
    while (1) {
	rt_sem_p(&sem_startRobot, TM_INFINITE);   //The task is blocked until the robot is started
        
       rt_mutex_acquire(&mutex_wd, TM_INFINITE);
       watchdog = wd;				  //the local var watchdog takes the value of the global var wd (updated in the ReceiveFromMonTask when the checkbox is checked or not)
       rt_mutex_release(&mutex_wd);

       rt_mutex_acquire(&mutex_robot, TM_INFINITE);
       if(watchdog==1) {                                 //if the Checkbox watchdog is checked
            cout << "Start robot with watchdog (";
            msgSend = robot.Write(robot.StartWithWD());  //Send a message to tell the robot to start with watchdog
            rt_sem_v(&sem_reload);			 //Unlock the ReloadTask to prevent the Robot from Time Out

       } else {                                          //if the Checkbox watchdog is not checked
           cout << "Start robot without watchdog (";
           msgSend = robot.Write(robot.StartWithoutWD());//Send a message to tell the robot to start without watchdog
       }
 	rt_mutex_release(&mutex_robot);
	    
       cout << msgSend->GetID();
       cout << ")" << endl;
	
       cout << "Movement answer: " << msgSend->ToString() << endl;
       WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon


       if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
           rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
           robotStarted = 1;
           rt_mutex_release(&mutex_robotStarted);
       } else if(msgSend->GetID() == MESSAGE_ANSWER_NACK) {       
           cout << "Robot not started :((((";
       }
       cout << endl << flush;

    }
}


// Monitor task ---------------------------------------------------------------------------------

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    int nb_err=0; //errors between the robot and the supervisor
	
    cout << "Start ReceiveFromMon Task " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        
	    if (nb_err <=3) {   //if the number of errors between the robot and the supervisor <=3 then we can receive messages                                              
		cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
            if (msgRcv->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) { //if a timeout from the robot is received, increase nb_err by one
                nb_err++;
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) { //if a normal message is received, nb_err=0
                nb_err = 0;
                rt_sem_v(&sem_openComRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
                nb_err = 0;
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){ // Battery Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_battery, TM_INFINITE);
                getBattery = 1;                                       //Global var put to 1 (battery checkbox checked)
                rt_mutex_release(&mutex_battery);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)){ //Watchdog Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_wd, TM_INFINITE);
	        wd = 0;                                               //Global var put to 0 (watchdog checkbox not checked)
	        rt_mutex_release(&mutex_wd);
	        rt_sem_v(&sem_startRobot);                            //Unlock StartRobotTask
            }
            else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){ //Watchdog Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_wd, TM_INFINITE);
	        wd = 1;						      //Global var put to 1 (watchdog checkbox checked)
	        rt_mutex_release(&mutex_wd);
	        rt_sem_v(&sem_startRobot);                            //Unlock StartRobotTask
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){      //Camera Task      
                nb_err = 0;
                rt_mutex_acquire(&mutex_camOpen, TM_INFINITE);  
                CamOpen=true;                                   //Global var put to true (Camera checkbox checked)	
                rt_mutex_release(&mutex_camOpen);
                rt_sem_v(&sem_openCam);				//Unlock Camera Task
            } 
            else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){     //Camera Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_camOpen, TM_INFINITE);
                CamOpen=false;					//Global var put to false (Camera checkbox not checked)
                rt_mutex_release(&mutex_camOpen);
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){ //Arena Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
                AskArena=true;					//Global var put to true (Arena button clicked)
                rt_mutex_release(&mutex_askArena);
                rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
                draw=true;					//Global var put to true (Authorize the drawing of the arena in the Camera Task)
                rt_mutex_release(&mutex_drawArena);
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){ //Arena Task
                nb_err = 0;
                rt_sem_v(&sem_arena);  //Unlock the Arena part to continue after chosing to CONFIRM or INFIRM the arena detected by the robot
                //draw already set to true in the MESSAGE_CAM_ASK_ARENA
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){ //Arena Task
                nb_err = 0;
                rt_sem_v(&sem_arena);   //Unlock the Arena part to continue after chosing to CONFIRM or INFIRM the arena detected by the robot
                rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
                draw=false;		//Global var put to false (Do not authorize the drawing of the arena in the Camera Task)
                rt_mutex_release(&mutex_drawArena);
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){ //Robot position Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_posCheck, TM_INFINITE);
                posCheck=true;		//Global var put to true (Robot position checkbox checked)
                rt_mutex_release(&mutex_posCheck);
            }
            else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){ //Robot position Task
                nb_err = 0;
                rt_mutex_acquire(&mutex_posCheck, TM_INFINITE);
                posCheck=false;		//Global var put to false (Robot position checkbox not checked)
                rt_mutex_release(&mutex_posCheck);
            }
            else if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)){ //Connection lost (Connection lost between the supervisor and the monitor)
                nb_err = 0;
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);  
                robotStarted = 0; //stop robot
                rt_mutex_release(&mutex_robotStarted);

                rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);  
                robot.Close(); //Close com robot
                rt_mutex_release(&mutex_comRobot);

                rt_mutex_acquire(&mutex_cam, TM_INFINITE);         
                camera->Close(); //close camera
                rt_mutex_release(&mutex_cam);

                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);         
                monitor.Close(); //close monitor
                rt_mutex_release(&mutex_monitor);
            }
        }
        else {
            cout << "3 Counter stop robot (timeout)" << __PRETTY_FUNCTION__ << endl << flush;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;                       // the number of errors between the robot and the supervisor >3, robot stopped
            rt_mutex_release(&mutex_robotStarted);
        }
        delete(msgRcv); // must be deleted manually, no consumer
    }
}


	    

//Already existing tasks ------------------------------------------------------------------------------------

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}



/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;
    int com_error;
	
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

	    
	    
        
    }
}


/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: \n" << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}




/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}
