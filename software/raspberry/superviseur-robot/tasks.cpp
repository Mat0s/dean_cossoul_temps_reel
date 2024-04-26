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
#define PRIORITY_TCHECKROBOT 20
#define PRIORITY_TCLOSEROBOT 26
#define PRIORITY_TSERVRESTART 30

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
	
	if (err = rt_sem_create(&sem_closeCam, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_startServ, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_restartServ, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_sem_create(&sem_closeRobot, NULL, 0, S_FIFO)) {
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
	if (err = rt_task_create(&th_closeRobot, "th_closeRobot", 0, PRIORITY_TCLOSEROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

	if (err = rt_task_create(&th_checkRobot, "th_checkRobot", 0, PRIORITY_TCHECKROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	if (err = rt_task_create(&th_serverRestart, "th_serverRestart", 0, PRIORITY_TSERVRESTART, 0)) {
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
    // getBattery
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    //Reload
    if (err = rt_task_start(&th_reload, (void(*)(void*)) & Tasks::ReloadTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }



	
	//Close robot
	if (err = rt_task_start(&th_closeRobot, (void(*)(void*)) & Tasks::CloseRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	//Camera (close)
	if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::CameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	
	//Check robot
	if (err = rt_task_start(&th_checkRobot, (void(*)(void*)) & Tasks::CheckRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
	//Server restart
	if (err = rt_task_start(&th_serverRestart, (void(*)(void*)) & Tasks::ServerRestartTask, this)) {
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

/**
 * @brief Thread handling server communication with the robot (get battery).
 */
void Tasks::GetBatteryTask(void *arg) {
    int rs;
    Message* batt_level;
    
    cout << "Start getBattery" << __PRETTY_FUNCTION__ << endl << flush;
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
 * @brief Thread handling server restart
 */
void Tasks::ServerRestartTask(void *arg) {
    int status;

    
    cout << "Start ServerTask" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/

    while (1) {
        rt_sem_p(&sem_restartServ, TM_INFINITE);
	//close com robot
	rt_sem_v(&sem_closeRobot);

	rt_sem_v(&sem_closeCam);

      	rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        cout << "Close server on port(5544)" << endl << flush;
        rt_mutex_release(&mutex_monitor);

	rt_sem_v(&sem_startServ);

    }
}


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
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    int err; 
    int nb_err=0;
	
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        
	if (nb_err ==0) {
		cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
        }
	    
        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST) && nb_err == 0) {
            nb_err++;
            rt_sem_v(&sem_restartServ);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            nb_err = 0;
            rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            err_Robot=0;
            rt_mutex_release(&mutex_comRobot);
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
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
			nb_err = 0;
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            getBattery = 1;
            rt_mutex_release(&mutex_battery);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)){
			nb_err = 0;
            rt_mutex_acquire(&mutex_wd, TM_INFINITE);
           wd = 0;
           rt_mutex_release(&mutex_wd);
           rt_sem_v(&sem_startRobot);

        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
			nb_err = 0;
            rt_mutex_acquire(&mutex_wd, TM_INFINITE);
           wd = 1;
           rt_mutex_release(&mutex_wd);
           rt_sem_v(&sem_startRobot);

	    rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            err = err_Robot;
            rt_mutex_release(&mutex_comRobot);
            
            if (err == 1){
                rt_sem_v(&sem_openComRobot);
            }
            else {
                rt_sem_v(&sem_startRobot);
            }
        }

        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            rt_mutex_acquire(&mutex_camOpen, TM_INFINITE);
            CamOpen=true;
            rt_mutex_release(&mutex_camOpen);
			rt_sem_v(&sem_openCam);
            
         } 
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            rt_mutex_acquire(&mutex_camOpen, TM_INFINITE);
            CamOpen=false;
            rt_mutex_release(&mutex_camOpen);
        }
         else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
            AskArena=true;
            rt_mutex_release(&mutex_askArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            rt_sem_v(&sem_arena);
            rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
            draw=true;
                        rt_mutex_release(&mutex_drawArena);

        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_sem_v(&sem_arena);
            rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
            draw=false;
                        rt_mutex_release(&mutex_drawArena);

        }
        
        delete(msgRcv); // mus be deleted manually, no consumer
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

	    
	    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        com_error = err_Robot;
        rt_mutex_release(&mutex_robotStarted);
        
        if (com_error == 1) {
            rt_sem_v(&sem_startRobot);
            rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            err_Robot = 0;
            rt_mutex_release(&mutex_comRobot);
        }
    }
}





/**
 * @brief Thread closing communication with the robot.
 */
void Tasks::CloseRobotTask(void *arg) {
    int status;
    int com_error;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeComRobot starts here                                                  */
    /**************************************************************************************/
    
    while (1) {
        rt_sem_p(&sem_closeRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
        com_error=err_Robot;
        rt_mutex_release(&mutex_comRobot);
        
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        cout << "Close  com ("<<flush;
        status = robot.Close();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;


        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);
 
        rt_mutex_acquire(&mutex_move, TM_INFINITE);
        move=MESSAGE_ROBOT_STOP;
        rt_mutex_release(&mutex_move);
        
              
    }
}























/**
 * @brief Close camera
 */
void Tasks::CameraTask(void *arg) {
    int status;
    int com_err;
   MessageImg * msgSend;
   camera = new Camera();
    
    
    cout << "Start CameraTask" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeComRobot starts here                                                  */
    /**************************************************************************************/
     rt_task_set_periodic(NULL, TM_NOW, 100000000); //100ms
    Arena arena;

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Wait for open camera" << __PRETTY_FUNCTION__ << endl << flush;
        rt_sem_p(&sem_openCam, TM_INFINITE);
        if (CamOpen) {
            rt_mutex_acquire(&mutex_cam, TM_INFINITE);       
            camera->Open();
            rt_mutex_release(&mutex_cam);
            while(CamOpen) {
                while(!AskArena) {

                    rt_mutex_acquire(&mutex_cam, TM_INFINITE);       
                    Img img = camera->Grab();
                    cout << "img taille : " << img.ToString() << endl << flush;
                    if (img.ToString()!="Image size: 0x0 (dim=2)") {
                        msgSend = new MessageImg(MESSAGE_CAM_IMAGE,&img);
                        if (!msgSend->GetImage()==NULL) {
                        WriteInQueue(&q_messageToMon, msgSend);
                        rt_mutex_release(&mutex_cam);
                    
                        if (draw) {
                            img.DrawArena(arena);
                        }
                        }
                    }
                
                }
                if (AskArena) {
                    
                    Img last_image = camera->Grab();
                       if (last_image.ToString()!="Image size: 0x0 (dim=2)") {
                             arena=last_image.SearchArena();

                            rt_sem_p(&sem_arena, TM_INFINITE);
                            if (draw) {
                                last_image.DrawArena(arena);
                                SavedArena=last_image;
                            }
                       }
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
 * @brief reload WD
 */
void Tasks::ReloadTask(void *arg) {
    int status;
    int err;
    Message * msgSend;
    cout << "Start RELOAD" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_reload, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000); //1s
    while (1) {
        rt_task_wait_period(NULL);


        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.ReloadWD());
        rt_mutex_release(&mutex_robot);


    }
}




/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    bool watchdog;
	Message * msgSend;

    cout << "StartRobotTask " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/


    while (1) {
     

        rt_sem_p(&sem_startRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_wd, TM_INFINITE);
       watchdog = wd;
       rt_mutex_release(&mutex_wd);


       // wd
       rt_mutex_acquire(&mutex_robot, TM_INFINITE);
       if(watchdog==1) {
           cout << "Start robot with watchdog (";

            msgSend = robot.Write(robot.StartWithWD());
            rt_sem_v(&sem_reload);

       } else {
           cout << "Start robot without watchdog (";

           msgSend = robot.Write(robot.StartWithoutWD());

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
 * @brief Thread handling the communication with the robot.
 */
void Tasks::CheckRobotTask(void *arg) {
    int rs, watchdog;
    int err_cmp;
    
    Message * status;
    cout << "Start CheckRobotTask" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000);
    
    err_cmp=0;

    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        rt_mutex_acquire(&mutex_wd, TM_INFINITE);
        watchdog = wd;
        rt_mutex_release(&mutex_wd);
        
        if (rs == 1 && watchdog == 1) {
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            status = robot.Write(new Message(MESSAGE_ROBOT_PING));
            rt_mutex_release(&mutex_robot);
            
            if (err_cmp >=3) {
                err_cmp=0;
                cout << "------Message : The communication with the robot has ended.------" << endl;
                rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
                err_Robot = 1;          
                rt_mutex_release(&mutex_comRobot);
                
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
               
                cout << "------Message : Closing the communication with the robot------" << endl;
                rt_sem_v(&sem_closeRobot);
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR));                    
            }
            else if (status->CompareID(MESSAGE_ANSWER_COM_ERROR)) {
               err_cmp++;
               cout << " status :  " << status->GetID()<< endl;
               cout << " err_cmp: " << err_cmp << endl;
               cout << " err_Robot :" << err_Robot << endl;
               cout << " rs : " <<robotStarted << endl << flush;
            }      
            else {
               err_cmp=0;
               cout << " err_cmp: " << err_cmp << endl;
            }
         
        }
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
