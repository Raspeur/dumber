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
#define PRIORITY_TGETBATTERY 18
#define PRIORITY_TMANAGEMENTCAMERA 19
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21

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
    if (err = rt_mutex_create(&mutex_stateCamera, NULL)) {
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
    if (err = rt_sem_create(&sem_stateCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_searchArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_validateArena, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_acquireBattery, "th_acquireBattery", 0, PRIORITY_TGETBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_managementCamera, "th_managementCamera", 0, PRIORITY_TMANAGEMENTCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_fluxVideo, "th_fluxVideo", 0, PRIORITY_TMANAGEMENTCAMERA, 0)) {
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
    if (err = rt_task_start(&th_acquireBattery, (void(*)(void*)) & Tasks::AcquireBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_managementCamera, (void(*)(void*)) & Tasks::CameraManagementTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_fluxVideo, (void(*)(void*)) & Tasks::FluxVideoTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_searchArena, (void(*)(void*)) & Tasks::SearchArena, this)) {
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
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if(msgRcv->CompareID(MESSAGE_CAM_OPEN) ||
                  msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_mutex_acquire(&mutex_updateCamera, TM_INFINITE);
            updateCamera = msgRcv->GetID();
            rt_mutex_release(&mutex_updateCamera);
            rt_sem_v(&sem_stateCamera);
        } else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_searchArena);
            arena_confirm = MESSAGE_CAM_ARENA_INFIRM;
        }else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) ||
                  msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            arena_confirm = msgRcv->GetID();
            rt_sem_v(&sem_validateArena);
        }
 

        delete(msgRcv); // must be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

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
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        // Check if the robot has started
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            // Update the robotStarted variable
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
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
    // Set the execution period to 100ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        // Acquire the state of the robot
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            // Acquire the movement to be done
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;

            // Send the movement to the robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread acquiring the battery level of the robot.
 */
void Tasks::AcquireBatteryTask(void *arg) {
    MessageBattery *msg;
    int rs;
    
    cout << "AcquireBattery " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    //Set the execution period to 500ms
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    //------------
    while (1)
    {
        // Wait for the next periodic release point
        rt_task_wait_period(NULL);
        // Acquire the state of the robot
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if(rs == 1)
        { 
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            
            cout << "Envoie valeur batterie toMon " << __PRETTY_FUNCTION__ << endl << flush;
            WriteInQueue(&q_messageToMon, msg); // msg will be deleted by sendToMon
        }
    }
}

/**
 * @brief Manages camera-related tasks. 
 *
 * This function is responsible for creating and deleting the camera object, depending on the updateCamera request.
 */
void Tasks::CameraManagementTask(void *arg)
{
    MessageID msg;
   
    cout << "Camera Management " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
   
    while(1)
    {
        cout << "Wait state Camera Update" << endl << flush;
        rt_sem_p(&sem_stateCamera, TM_INFINITE);
        
        // Acquisition of the updateCamera message ID (block the access to the updateCamera)
        rt_mutex_acquire(&mutex_updateCamera, TM_INFINITE);
        msg = (MessageID)updateCamera;
        // Realease the access to the updateCamera message ID
        rt_mutex_release(&mutex_updateCamera);

        // Update the state of the camera (block the access to the camera object)
        rt_mutex_acquire(&mutex_stateCamera, TM_INFINITE);
        if(msg == MESSAGE_CAM_OPEN)
        {
            // Creation of a new object Camera
            cam = new Camera(sm, 10);
            cam->Open();
        }
        else if(msg == MESSAGE_CAM_CLOSE)
        {
            // Close and delete the previous created Camera object
            cam->Close();
            delete cam;
        };
        // Update the state of the camera
        stateCamera = (MessageID)updateCamera;
        rt_mutex_release(&mutex_stateCamera);
    }
}

/**
 * @brief Manages video stream-related tasks.
 *
 * This function is responsible for capturing frames from the camera, depending on its state,
 * and sending them to the monitor.
 */
void Tasks::FluxVideoTask(void *arg)
{
    Img *image;
    MessageImg *msgimg;
    // Jpg image_compressed
    MessageID stateCam = MESSAGE_CAM_CLOSE;
    cout << "FluxVideo" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1)
    {
        // Set the execution period to 100ms
        rt_task_set_periodic(NULL, TM_NOW, 100000000);
        // Wait for the next periodic release point
        rt_task_wait_period(NULL);
        // Acquisition of the state of the camera (block the access to the camera object)
        rt_mutex_acquire(&mutex_stateCamera, TM_INFINITE);
        stateCam = (MessageID)stateCamera;

        if(stateCam == MESSAGE_CAM_OPEN && )
        {
            // Capture an image
            image = new Img(cam->Grab());
            if(image != NULL)
            {
                rt_mutex_acquire(&mutex_arenaValidation, TM_INFINITE);
                if(arena_valid != NULL)
                {
                    image->DrawArena(arena_valid);
                }
                rt_mutex_release(&mutex_arenaValidation);
                // Create a new message with the image
                msgimg = new MessageImg(MESSAGE_CAM_IMAGE, image);
                // Send the image/message to the monitor
                WriteInQueue(&q_messageToMon, msgimg); // msg will be deleted by sendToMon
            }            
        }
        // Realease the access to the camera object
        rt_mutex_release(&mutex_stateCamera);
    }
}
/**
 * @brief Search the arena in the image
 */
void Tasks::SearchArena(void *arg)
{
    bool arenaFound = false;
    Arena *arena;
    Img *image_arena;
    MessageImg *msgimg;
    Message * msg_to_mon;
    cout << "Start Research of the arena" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    // The task starts here
    while (1)
    {
        // Wait for the next release point
        rt_sem_p(&sem_searchArena, TM_INFINITE);
        cout << "Search the arena" << endl << flush;
        // Acquisition of the state of the camera (block the access to the camera object)
        rt_mutex_acquire(&mutex_stateCamera, TM_INFINITE);
        if(cam->IsOpen())
        {
            // Capture an image
            image_arena = new Img(cam->Grab());

            // Check if the image is empty
            if(image_arena == NULL)
            {
                // Create a new message with the nack
                msg_to_mon = new Message(MESSAGE_ANSWER_NACK);
                WriteInQueue(&q_messageToMon, msg_to_mon);
            }
            else
            {
                // Search the arena in the image
                arena = new Arena (image_arena->SearchArena());
                if(!(arena->IsEmpty()))
                {
                    // Create a new message with the ack
                    msg_to_mon = new Message(MESSAGE_ANSWER_ACK);
                    WriteInQueue(&q_messageToMon, msg_to_mon);
                
                    // Draw the arena in the image
                    image_arena->DrawArena(arena);
                    
                    // Create a new message with the image
                    msgimg = new MessageImg(MESSAGE_CAM_IMAGE, image_arena);
                    // Send the image/message to the monitor
                    WriteInQueue(&q_messageToMon, msgimg); // msg will be deleted by sendToMon

                    // Wait for the next release point
                    rt_sem_p(&sem_validateArena, TM_INFINITE);
                    
                    rt_mutex_acquire(&mutex_arenaValidation, TM_INFINITE);
                    // Check if the arena is valid
                    if(arena_confirm)
                    {
                        arena_valid = arena;
                    }
                    rt_mutex_release(&mutex_arenaValidation);
                }
            }
        }
        else
        {
            // Create a new message with the nack
            msg_to_mon = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msg_to_mon);
        }
        // Realease the access to the camera object
        rt_mutex_release(&mutex_stateCamera);
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
