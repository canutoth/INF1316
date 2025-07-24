/****************************************************************
*                                                               *
*   T1: Simulação de um Sistema de Controle de Tráfego Aéreo    *
*                                                               *
*             Luana Pinho Bueno Penha    - 2312082              *
*             Theo Jesus Canuto de Sousa - 2311293              *
*                                                               *
*                       INF1316 - 2025.1                        *
*                                                               *
****************************************************************/


#include <stdio.h> // printf, fprintf, perror
#include <stdlib.h> // exit, atoi, rand, srand, malloc, free
#include <unistd.h> // fork, getpid, sleep, usleep, kill, raise
#include <sys/ipc.h> // IPC_PRIVATE (shared memory)
#include <sys/shm.h> // shmget, shmat, shmdt, shmctl (shared memory)
#include <sys/wait.h> // wait, waitpid
#include <sys/stat.h> // S_IRUSR, S_IWUSR (shared memory permissions)
#include <signal.h> // signal, SIGTSTP, SIGQUIT, SIGINT, SIGSTOP, SIGCONT, SIGUSR1, SIGUSR2
#include <stdbool.h> // (true, false)
#include <time.h> // time (rand seed)
#include <math.h> // fabsf (float absolute value)


#define MAX_AIRCRAFT 40
#define COORD_TARGET 0.5f
#define BASE_SPEED 0.05f // for x
#define LANDED 1
#define DEAD 2
#define PAUSED 3
#define FLYING 4

struct Aircraft {
    pid_t pid;
    int dir_x; // W = 1 or E = -1 bc that dictates if they increase or decrease x when moving
    float x, y; // loc starts x=1 for E or x=0 for W
    int delay; // 0, 1 or 2s
    int landing; // 27 or 6 for E or 18 or 3 for W
    float speed_x; // default is 0.05
    float speed_y; // rectriangle based on random value of y
    int status; // 1- landed 2- dead 3- paused 4- flying
}; typedef struct Aircraft aircraft;

int num_aircrafts; // tot aircrafts entered by user
aircraft *ac; // array of aircrafts (shared memory)
int shm_id; // shared memory id
int curr_ac_idx; // index of the current aircraft being processed
int num_dead = 0; // number of aircrafts that were killed
int num_landed = 0; // number of aircrafts that landed successfully
bool paused = false; // if user paused or not
pid_t parent_pid; // parent process id (scheduler)
int old_status, old_landing; // status and landing of aircraft before changing them

aircraft create_aircraft(pid_t pid); // creates aircraft with random values
void sig1_handler(int signal); // SIGUSR1 handler (pause/resume aircraft)
void sig2_handler(int signal); // SIGUSR2 handler (change aircraft runway)
void pause_resume_handler(int signal); // SIGTSTP handler (pause/resume all aircrafts)
void kill_handler(int signal); // SIGINT handler (kill all aircrafts)
void stop_program_handler(int signal); // SIGQUIT handler (stop program)
bool collision_risk(int idx, int ignoreIdx); // check if aircraft idx is at risk of collision with any other aircrafts (except ignoreIdx)
bool too_close(int i, int j); // check if aircrafts i and j are too close to each other (0.1f)
void treat_collisions(int idxA, int idxB); // treat collision between aircrafts idxA and idxB
void detect_collisions(); // detect collisions between aircrafts

int main(int argc, char *argv[]) {

    parent_pid = getpid();
    
    if (argc != 2) {
        fprintf(stderr, "Correct call is %s <num_aircrafts>\n", argv[0]);
        exit(EXIT_FAILURE);
    } // check if the number of arguments is correct

    num_aircrafts = atoi(argv[1]); // convert string to int

    if (num_aircrafts <= 0 || num_aircrafts > MAX_AIRCRAFT) {
        fprintf(stderr, "Number of aircrafts must be between 1 and %d\n", MAX_AIRCRAFT);
        exit(EXIT_FAILURE);
    } // check if the number of aircrafts is valid

    shm_id = shmget(IPC_PRIVATE, sizeof(aircraft)*num_aircrafts, IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR); // create shared memory segment
   
    if (shm_id < 0) {
        perror("shmget");
        exit(EXIT_FAILURE);
    } // check if shared memory segment was created successfully

    ac = (aircraft *)shmat(shm_id, NULL, 0); // attach shared memory segment to process address space
    
    if (ac == (void *)-1) {
        perror("shmat");
        exit(EXIT_FAILURE);
    } // check if shared memory segment was attached successfully

    signal(SIGTSTP, pause_resume_handler); // SIGTSTP handler (pause/resume all aircrafts)
    signal(SIGQUIT, stop_program_handler); // SIGQUIT handler (stop program)
    signal(SIGINT, kill_handler); // SIGINT handler (kill all aircrafts)

    printf("Press ctrl+Z to pause/resume all aircrafts.\n");
    printf("Press ctrl+C to kill all aircrafts.\n");
    printf("Press ctrl+\\ to stop the program.\n\n");

    for (int i = 0; i < num_aircrafts; i++){

        pid_t pid = fork(); // create child process for each aircraft
        
        if (pid < 0) { // fork failed

            perror("fork");
            exit(EXIT_FAILURE);

        } else if (pid == 0) { // child process, aka aircraft per se
            
            srand(time(NULL) ^ getpid()); // new seed for random values (XOR)
            ac[i] = create_aircraft(getpid()); // create aircraft with random values
            curr_ac_idx = i; // set current aircraft index to the one created
            
            signal(SIGUSR1, sig1_handler); // SIGUSR1 handler (pause/resume aircraft)
            signal(SIGUSR2, sig2_handler); // SIGUSR2 handler (change aircraft runway)

            printf("aircraft %d data: delay: %ds, side: %d,"
            " x: %.2f and y: %.2f, runway: %d, speed_x: %.2f, status: %d\n\n",
            ac[curr_ac_idx].pid,
            ac[curr_ac_idx].delay,
            ac[curr_ac_idx].dir_x,
            ac[curr_ac_idx].x,
            ac[curr_ac_idx].y,
            ac[curr_ac_idx].landing,
            ac[curr_ac_idx].speed_x,
            ac[curr_ac_idx].status); // print aircraft data

            raise(SIGSTOP); // pause aircraft until scheduler is ready to process it
            
            sleep(ac[curr_ac_idx].delay); // delay before starting to fly
            
            while (1) { // main loop of the aircraft

                switch (ac[curr_ac_idx].status) { // check aircraft status

                    case PAUSED: // aircraft is paused
                    
                        printf("aircraft %d PAUSED at (%.2f, %.2f), status: %d\n",
                            ac[curr_ac_idx].pid, ac[curr_ac_idx].x, ac[curr_ac_idx].y, ac[curr_ac_idx].status);
                          
                        break;
                        
                    case FLYING: // aircraft is flying
                        ac[curr_ac_idx].x += ac[curr_ac_idx].dir_x * ac[curr_ac_idx].speed_x; // move aircraft in x direction
                        ac[curr_ac_idx].y += ac[curr_ac_idx].speed_y; // move aircraft in y direction
                        
                        if (fabsf(ac[curr_ac_idx].x-COORD_TARGET)<=0.01 && fabsf(ac[curr_ac_idx].y-COORD_TARGET)<=0.01) { // check if aircraft reached target coordinates
                            ac[curr_ac_idx].status = LANDED; // set aircraft status to LANDED
                            ac[curr_ac_idx].x = COORD_TARGET; // set aircraft x coordinate to target
                            ac[curr_ac_idx].y = COORD_TARGET; // set aircraft y coordinate to target
                            break;
                        }
                        
                        // aircraft didn't land yet, so keep going

                        detect_collisions(); // check for collisions with other aircrafts

                        printf("aircraft %d - x: %.2f, y: %.2f, runway: %d\n",
                        ac[curr_ac_idx].pid,
                        ac[curr_ac_idx].x,
                        ac[curr_ac_idx].y,
                        ac[curr_ac_idx].landing); // print aircraft current position and runway

                        break;
                        
                    case LANDED:
                        
                        printf("aircraft %d landed at runway %d in (x,y) = (%.2f,%.2f)\n",
                        ac[curr_ac_idx].pid,
                        ac[curr_ac_idx].landing,
                        ac[curr_ac_idx].x,
                        ac[curr_ac_idx].y); // print aircraft landed message
                        
                        exit(EXIT_SUCCESS); // exit aircraft process

                        break;
                        
                    default: // aircraft is dead
                        break;

                }

                sleep(1); // sleep for 1 second to simulate aircraft movement
                    
            }
                    
        }
    
    }

    usleep(100000); // wait for all aircrafts to be created

    int curr = 0; // current aircraft index being processed

    while (1) { // main loop of the scheduler: round robin + checking if it's done

        // check if all aircrafts are either LANDED or DEAD

        bool done = true; 

        for (int i = 0; i < num_aircrafts; i++) {
            if (ac[i].status != LANDED  && ac[i].status != DEAD) {
                done = false;
                break;
            }
        }
        
        if (done) break; // all aircrafts are either LANDED or DEAD, exit loop

        // not all aircrafts are done, so keep going

        if(!paused){ // if not paused by user, process aircrafts in round robin fashion
            
            if(ac[curr].status == FLYING || ac[curr].status == PAUSED){ // if aircraft is flying or paused, process it
                
                kill(ac[curr].pid, SIGCONT);
                sleep(1);
                kill(ac[curr].pid, SIGSTOP);
                
            }
    
            curr = (curr + 1) % num_aircrafts; // round robin: go to next aircraft

        }

    
    }

    for (int i = 0; i < num_aircrafts; i++) kill(ac[i].pid, SIGCONT); // wake up all aircrafts to finish them
    for (int i = 0; i < num_aircrafts; i++) wait(NULL); // wait for all aircrafts to finish

    for (int i = 0; i < num_aircrafts; i++) { // filling final stats
        if (ac[i].status == DEAD) num_dead++;
        else if (ac[i].status == LANDED) num_landed++;
    }
    
    shmdt(ac); // detach shared memory segment from process address space
    shmctl(shm_id, IPC_RMID, NULL); // remove shared memory segment

    // printing final stats
    printf("\n\nFINAL STATS:\n\n"); 

    printf("successfully landed aircrafts: %d\n", num_landed);
    printf("killed aircrafts: %d\n", num_dead);
    float pct = num_landed*100/num_aircrafts;
    printf("%.2f%% of the aircrafts landed successfully.\n", pct);
    
    return 0;
}

aircraft create_aircraft(pid_t pid){ // create aircraft with random values
    aircraft a; // create aircraft struct

    if (rand() % 2) { // randomize direction (1 for W, -1 for E)
        a.dir_x = 1;
        a.x = 0.0f;
        a.landing = (rand() % 2) ? 18 : 3;
    } else {
        a.dir_x = -1;
        a.x = 1.0f;
        a.landing = (rand() % 2) ? 27 : 6;
    }

    a.y = (float)(rand() % 11) / 10.0f; // randomize y coordinate (0.0 to 1.0)
    a.delay = rand() % 3; // randomize delay (0, 1 or 2 seconds)
    a.pid = pid; // set aircraft pid to received parameter
    a.speed_x = BASE_SPEED; // set speed_x to default value (0.05)
    float delta_x = COORD_TARGET; // distance to target in x direction (0.5 for W or 0.5 for E)
    float delta_y = COORD_TARGET - a.y; // distance to target in y direction (0.5 - y)
    a.speed_y = (delta_y / delta_x) * a.speed_x; // calculate speed_y based on distance to target and speed_x
    a.status = FLYING; // set aircraft status to FLYING - every aircraft starts flying

    return a; // return created aircraft

}

void sig1_handler(int signal) { // SIGUSR1 handler (pause/resume aircraft)
    
    ac[curr_ac_idx].status = (ac[curr_ac_idx].status == PAUSED) ? FLYING : PAUSED; // toggle aircraft status between PAUSED and FLYING

    return;
}

void sig2_handler(int signal) { // SIGUSR2 handler (change aircraft runway)
    switch (ac[curr_ac_idx].landing) { // toggle aircraft runway between 27 and 6 for E or 18 and 3 for W
        case 18:
            ac[curr_ac_idx].landing = 3;
            break;
        case 3:
            ac[curr_ac_idx].landing = 18;
            break;
        case 27:
            ac[curr_ac_idx].landing = 6;
            break;
        default:
            ac[curr_ac_idx].landing = 27;
            break;
    }

    return;
}

void pause_resume_handler(int signal) { // SIGTSTP handler (pause/resume all aircrafts)

    if (!paused) { // if not paused by user, pause all aircrafts
        printf("\npausing all aircrafts...\n");
        for (int i = 0; i < num_aircrafts; i++) {
            if (ac[i].status == FLYING || ac[i].status == PAUSED) {
                kill(ac[i].pid, SIGSTOP);
                printf("pausing aircraft %d\n", ac[i].pid);
                usleep(100000);
            }
        }
        paused = true; // set paused to true
    } else { // if paused by user, resume all aircrafts
        printf("\nresuming all aircrafts...\n");
        for (int i = 0; i < num_aircrafts; i++) {
            if (ac[i].status == FLYING || ac[i].status == PAUSED) {
                kill(ac[i].pid, SIGCONT);
                printf("resuming aircraft %d\n", ac[i].pid);
                usleep(100000);
            }
        }
        paused = false; // set paused to false
    }

    usleep(100000);
    return;
}

void kill_handler(int signal){ // SIGINT handler (kill all aircrafts)
    
    for(int i=0; i<num_aircrafts; i++) { // kill all aircrafts that haven't landed yet
        if(ac[i].status != LANDED) {
            ac[i].status = DEAD;
            kill(ac[i].pid, SIGKILL);
        }
    }
    
    printf("\nkilling all aircrafts in traffic.\n");
    return;
}

void stop_program_handler(int signal){ // SIGQUIT handler (stop program)

    if(getpid()==parent_pid) printf("\nfinishing process...\n"); // if parent process, print message (avoids duplicate messages)
    exit (0); // exit program

}

bool collision_risk(int idx, int ignoreIdx) { // check if aircraft idx is at risk of collision with any other aircrafts (except ignoreIdx)
    for (int j = 0; j < num_aircrafts; j++) { // iterate through all aircrafts
        if (j == idx || j == ignoreIdx) // // skip current aircraft and ignore aircraft
        continue;
        if (ac[j].status == LANDED || ac[j].status == DEAD) // skip aircrafts that are already landed or dead
        continue;
        if (ac[j].landing != ac[idx].landing) // skip aircrafts that are not landing on the same runway
        continue;
        if (fabsf(ac[idx].x - ac[j].x) <= 0.1f &&
        fabsf(ac[idx].y - ac[j].y) <= 0.1f) { // check if aircrafts are too close to each other
            return true; // if they are, return true (collision risk)
        }
    }
    return false; // if no collision risk found, return false
}

bool too_close(int i, int j) { // check if aircrafts i and j are too close to each other (0.1f)
    return fabsf(ac[i].x - ac[j].x) <= 0.1f && fabsf(ac[i].y - ac[j].y) <= 0.1f; // check if aircrafts are still too close to each other
}

void treat_collisions(int idxA, int idxB) { // treat collision between aircrafts idxA and idxB
    
    old_landing = ac[idxA].landing; // save old landing of aircraft A to check if it changed
    kill(ac[idxA].pid, SIGUSR2); // try swaping runways
    printf("changing runway of %d (SIGUSR2)\n", ac[idxA].pid);
    usleep(100000);
    if (ac[idxA].landing != old_landing) { // check if runway changed
        printf("scheduler: SIGUSR2 received, runway changed from %d to %d\n",
            old_landing, ac[idxA].landing);
    } else { // if runway didn't change, print error message
        fprintf(stderr, "scheduler: ERROR! aircraft %d didn't process SIGUSR2\n",
            ac[idxA].pid);
    }
    
    if (!collision_risk(idxA, -1)) return; // worked! byebye
    
    // didnt work:
    old_landing = ac[idxA].landing;
    kill(ac[idxA].pid, SIGUSR2); // get back to old runway
    printf("changing runway of %d didn't work. changing it back\n", ac[idxA].pid);
    usleep(100000);
    if (ac[idxA].landing != old_landing) { // check if runway changed
        printf("scheduler: SIGUSR2 received, runway changed from %d to %d\n",
            old_landing, ac[idxA].landing);
    } else { // if runway didn't change, print error message
        fprintf(stderr, "scheduler: ERROR! aircraft %d didn't process SIGUSR2\n",
            ac[idxA].pid);
    }

    old_status = ac[idxA].status; // save old status of aircraft A to check if it changed
    kill(ac[idxA].pid, SIGUSR1);
    printf("pausing aircraft %d (SIGUSR1)\n", ac[idxA].pid);
    usleep(100000);
    if (ac[idxA].status != old_status) { // check if status changed
        printf("scheduler: SIGUSR1 received, status changed from %d to %d\n",
            old_status, ac[idxA].status);
    } else { // if status didn't change, print error message
        fprintf(stderr, "scheduler: ERROR! aircraft %d didn't process SIGUSR1. status' still %d\n",
            ac[idxA].pid, ac[idxA].status);
    }
    
    while (too_close(idxA, idxB)) { // while aircrafts are too close to each other
        printf("im still too close [%d status %d, %d status %d]\n", ac[idxA].pid, ac[idxA].status, ac[idxB].pid, ac[idxB].status);
        printf("aircraft %d PAUSED at (%.2f, %.2f)\n",
                            ac[curr_ac_idx].pid, ac[curr_ac_idx].x, ac[curr_ac_idx].y);
        if (collision_risk(idxA, idxB)) { // check if any other aircraft is about to collide with the paused one 
            printf("pausing or swaping runways didn't work. unavoidable SIGKILL on aircraft %d\n", ac[idxA].pid);
            ac[idxA].status = DEAD; // set aircraft status to DEAD bc the handlers were not able to fix the problem
            
            kill(ac[idxA].pid, SIGKILL); // kill aircraft
            return;
        }

        if (ac[idxB].status == DEAD || ac[idxB].status == LANDED){ // if aircraft B is dead or landed, we can stop checking for collisions
            break;
        }

        usleep(100000);
    }

    // aircrafts are not too close anymore, so we can resume aircraft A
    printf("im not too close anymore [%d status %d, %d status %d]\n", ac[idxA].pid, ac[idxA].status, ac[idxB].pid, ac[idxB].status);
    
    // resume A
    old_status = ac[idxA].status; // save old status of aircraft A to check if it changed
    kill(ac[idxA].pid, SIGUSR1);
    printf("resuming aircraft %d (SIGUSR1)\n", ac[idxA].pid);
    usleep(100000);
    if (ac[idxA].status != old_status) { // check if status changed
        printf("scheduler: SIGUSR1 received, status changed from %d to %d\n",
            old_status, ac[idxA].status);
    } else { // if status didn't change, print error message
        fprintf(stderr, "scheduler: ERROR! aircraft %d didn't process SIGUSR1. status' still %d\n",
            ac[idxA].pid, ac[idxA].status);
    }

    return;
}

void detect_collisions() { // detect collisions between aircrafts
    for (int j = 0; j < num_aircrafts; j++) { // iterate through all aircrafts
        if (j == curr_ac_idx || j < curr_ac_idx) // skip current aircraft and already processed aircrafts
            continue;

        if (ac[curr_ac_idx].landing != ac[j].landing) // skip aircrafts that are not landing on the same runway
            continue;
        if (ac[j].status == LANDED || ac[j].status == DEAD) // skip aircrafts that are already landed or dead
            continue;

        if (fabsf(ac[curr_ac_idx].x - ac[j].x) <= 0.1f &&
            fabsf(ac[curr_ac_idx].y - ac[j].y) <= 0.1f) { // check if aircrafts are too close to each other

            printf("oops! aircrafts %d and %d might collide. let's fix it...\n",
                   ac[curr_ac_idx].pid, ac[j].pid);

            treat_collisions(curr_ac_idx, j); // treat collision between aircrafts curr_ac_idx and j
        }
    }
}