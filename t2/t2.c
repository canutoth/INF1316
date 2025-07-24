/****************************************************************
*                                                               *
*           T2: Gerenciamento de memória virtual (GMV)          *
*                                                               *
*               Luana Pinho Bueno Penha  - 2312082              *
*             Theo Jesus Canuto de Sousa - 2311293              *
*                                                               *
*                       INF1316 - 2025.1                        *
*                                                               *
****************************************************************/

#define _POSIX_C_SOURCE 200809L   /* habilita funções POSIX (kill, usleep, etc.) */
#define _DEFAULT_SOURCE            /* habilita GNU + POSIX na glibc */
#include <stdio.h> // printf, perror, etc
#include <stdlib.h> // exit, atoi, etc
#include <unistd.h> // fork, getpid, kill, usleep, etc
#include <string.h> // strcmp, etc
#include <sys/shm.h> // shmget, shmat, shmctl
#include <sys/wait.h> // waitpid
#include <signal.h> // SIGKILL, SIGSTOP, SIGCONT
#include <stdbool.h> // bool type
#include <time.h> // time, srand, rand
#include <limits.h> // UNLONG_MAX
#include <fcntl.h> // fcntl, O_NONBLOCK
#include <errno.h> // errno, EAGAIN, EWOULDBLOCK

#define NUM_PROCESSES 4 // number of processes
#define NUM_FRAMES 16 // RAM has 16 frames
#define NUM_PAGES 32 // 32 pages for each process
#define PIPE_READ 0 // read end of the pipe
#define PIPE_WRITE 1 // write end of the pipe
#define QUANTUM 100000 // quantum time for each process in microseconds
#define PAGE_FAULT_DELAY 50000 // punishment delay for page fault in microseconds

typedef enum { //the four algorithms
    NRU, // Not Recently Used
    SEGUNDA_CHANCE, // Second Chance
    LRU, // Least Recently Used
    WORKING_SET // Working Set
} Algorithm;

typedef struct  {
    int frame; //which frame it's in 
    bool present; // page in RAM? 
    bool modified; // M (slide page 96)
    bool referenced; // R (slide page 96)
    unsigned char aging_counter; // 8 bits de aging (simulando LRU)
    unsigned long last_access; // timestamp of last reference
} PageEntry; //Page entry of a table of processes (TP). PageTableEntry

typedef struct { 
    int process_id; // which process is it
    int virtual_address; // page?
} ProcessMessage; 

typedef struct {
    int page_number; //number of the page she's in
    char access_type; // 'R' or 'W'
} ProcessAccess;

typedef struct {
    int proc; // índice de 0 a NUM_PROCESSES-1
    int page; // índice de 0 a NUM_PAGES-1
} FrameMap;

// Function prototypes
void select_NRU(int *out_p, int *out_pg);
void select_2nCh(int *out_p, int *out_pg);
void select_LRU(int *out_p, int *out_pg);
void select_WS(int *out_p, int *out_pg, int k);
void initialize_memory(); 
void GMV_Process();
int find_free_frame();
void allocate_frame(int proc, int page, int frame, char access_type);
void print_summary();
static inline void update_aging_counters(void);


// Global variables
FrameMap frame_map[NUM_FRAMES]; // map of frames to processes and pages
int shm_pf; // id of the shared memory for page fault flags
int shmid; // id of the shared memory
int k = 0; // parameter for Working Set algorithm, default is 0
int num_rounds = 0; // number of rounds given by the user
bool free_frames[NUM_FRAMES]; // number of free_frames. Initializes with total frames being free
Algorithm current_Algorithm; // currentProgress given by the user
pid_t processes[NUM_PROCESSES] = {0,0,0,0}; // all my process together
PageEntry (*shared_page_tables)[NUM_PAGES]; // TP for each process (shared memory)
const char* processesPaths[NUM_PROCESSES] = {"acessos_P1.txt", "acessos_P2.txt","acessos_P3.txt", "acessos_P4.txt"}; // paths to the access files of each process
int pipes[NUM_PROCESSES][2]; // pipes for communication of each process
unsigned long clock_counter = 0; // global clock counter for WS algorithm
int clock_hand = 0; // global counter for 2nd chance
bool* had_page_fault; // shared memory to indicate if a process had a page fault
int swap_writes = 0; // number of writes to swap
int page_faults[NUM_PROCESSES] = {0}; // number of page faults for each process
static volatile sig_atomic_t keep_running = 1; // flag to control the GMV process
static void term_handler(int sig); // signal handler to terminate the GMV process



int main(int argc, char *argv[]) {
    
    // checking if the user gave the right arguments
    if (argc < 3) {
        printf("Invalid number of arguments\n");
        exit(EXIT_FAILURE); 
    }

    num_rounds = atoi(argv[2]); // the user will give the number of rounds

    // getting the process given by the user
    if (strcmp(argv[1], "NRU") == 0) {
        current_Algorithm = NRU;
    } else if (strcmp(argv[1], "2nCH") == 0) { 
        current_Algorithm = SEGUNDA_CHANCE;
    } else if (strcmp(argv[1], "LRU") == 0) { 
        current_Algorithm = LRU;
    } else if (strcmp(argv[1], "WS") == 0){ 
        current_Algorithm = WORKING_SET;
        if (argc > 3) k = atoi(argv[3]);
        else {
            fprintf(stderr, "Missing parameter k for Working Set.\n");
            exit(EXIT_FAILURE);
        }
    } else {
        printf("Invalid argument: %s\n", argv[1]);
        exit(EXIT_FAILURE);
    }

    
    shm_pf = shmget(IPC_PRIVATE, sizeof(bool) * NUM_PROCESSES, IPC_CREAT | 0644); // shared memory for page fault flags
    
    // checking if the shared memory was created successfully
    if (shm_pf < 0) {
        perror("shmget");
        exit(EXIT_FAILURE);
    }

    had_page_fault = shmat(shm_pf, NULL, 0); // attaches the shared memory to the process address space

    // checking if the shared memory was attached successfully
    // had_page_fault will be used to indicate if a process had a page fault
    if (had_page_fault == (void*) -1) {
        perror("shmat");
        exit(EXIT_FAILURE);
    }

    // initializing the had_page_fault array to false 

    for (int i = 0; i < NUM_PROCESSES; ++i) had_page_fault[i] = false;
    
    shmid = shmget(IPC_PRIVATE, sizeof(PageEntry) * NUM_PROCESSES * NUM_PAGES, IPC_CREAT | 0644); // shared memory for page tables of all processes

    // checking if the shared memory was created successfully
    if (shmid < 0) {
        perror("shmget");
        exit(EXIT_FAILURE);
    }

    shared_page_tables = (PageEntry (*)[NUM_PAGES]) shmat(shmid, (void*)0, 0); // creates the shared vector (has the pages of all processes)
    
    // checking if the shared memory was attached successfully
    if (shared_page_tables == (void*) -1) {
        perror("shmat");
        exit(EXIT_FAILURE);
    }

    // creating pipes for each process
    for (int i = 0; i < NUM_PROCESSES; i++) {
        if (pipe(pipes[i]) == -1) {
            perror("pipe");
            for (int j = 0; j < i; j++) {
                close(pipes[j][PIPE_READ]);
                close(pipes[j][PIPE_WRITE]);
            }
            shmdt(shared_page_tables);
            shmctl(shmid, IPC_RMID, NULL);
            shmctl(shm_pf, IPC_RMID, NULL);
            exit(EXIT_FAILURE);
        }
    }

    initialize_memory(); // initializing frames

    // creating the processes
    for (int i = 0; i < NUM_PROCESSES; i++){ 
        pid_t pid = fork(); 
   
        if (pid < 0) { // error in fork
            perror("fork"); 
            exit(EXIT_FAILURE); 
        } 

        if(pid == 0){ // child
            printf("Child %d initialized \n", i + 1); 
            close(pipes[i][PIPE_READ]); // closes the read end of the pipe in the child process
            FILE* file = fopen(processesPaths[i], "r"); // opens the access file for the child process
            if (file == NULL) { // error opening the file
                perror("Error opening access file in the child process");
                exit(EXIT_FAILURE);
            }

            while(1){
               // create a process, read from the file: (the number of the page, the type of acess (R to read, W to write))
                ProcessAccess acesso;
                if (fscanf(file, "%d %c\n", &acesso.page_number, &acesso.access_type) != 2) {
                    fseek(file, 0, SEEK_SET);
                    fscanf(file, "%d %c", &acesso.page_number, &acesso.access_type); 
                }
                raise(SIGSTOP);
                // writting for the parent to read latter (if doesnt work, error)
                if (write(pipes[i][PIPE_WRITE], &acesso, sizeof(ProcessAccess)) == -1) { // writes the acess to the pipe
                perror("Error writing to the child's pipe");
                exit(EXIT_FAILURE);
                }
            }

            fclose(file); 
            exit(EXIT_SUCCESS); 
        }
        
        // parent process
        else{
            processes[i] = pid;
            close(pipes[i][PIPE_WRITE]); // closes the write end of the pipe in the parent process
        }
    }

    pid_t gmv_pid = fork(); // creating the GMV process
    if (gmv_pid == 0) {
        GMV_Process();
        exit(EXIT_SUCCESS);
    }

    printf("\n\n == Children initialized successfully! ==\n\n");
    srand(time(NULL)); 
    printf("\n== Initializing Round-Robin! ==\n");

    int rounds = 0;

    while (rounds < num_rounds) { // loop for the number of rounds given by the user
        printf("\n[Scheduler] Round %d\n", rounds + 1);
        clock_counter++; // incrementing the global clock counter for WS algorithm
        update_aging_counters(); // updating aging counters for LRU algorithm
        for (int i = 0; i < NUM_PROCESSES; i++) { // iterating through the processes
            printf("Executing the P%d (pid = %d).\n", i + 1, processes[i]);

            // Round-robin scheduling with penalty for processes that had page faults
            kill(processes[i], SIGCONT);

            if (had_page_fault[i]) {
                printf("[Scheduler] P%d incurred page-fault, adding %dµs penalty\n",
                    i+1, PAGE_FAULT_DELAY);
                fflush(stdout);
                usleep(PAGE_FAULT_DELAY); // adding penalty for page fault
                had_page_fault[i] = false;  // reset the page fault flag
            }
            
            usleep(QUANTUM); // quantum time for each process
            
            kill(processes[i], SIGSTOP);
        }
        rounds++;
        printf("number of rounds: %d\n", rounds); 
    }
    
    
    kill(gmv_pid, SIGTERM); // sending termination signal to GMV process
    waitpid(gmv_pid, NULL, 0); // waiting for GMV process to finish
    
    return 0;

}

// Function to select a page using the NRU (Not Recently Used) algorithm
// It classifies pages into four classes based on their referenced and modified bits
void select_NRU(int *out_p, int *out_pg) {
    static int call_counter = 0; // Function call counter

    // Page categories
    int classes[4][NUM_FRAMES]; // Array to store frames by class
    int counts[4] = {0}; // Page count per class
    
    // Classify all pages in memory
    for (int f = 0; f < NUM_FRAMES; f++) {
        if (frame_map[f].proc >= 0) { // Frame is in use
            int proc = frame_map[f].proc;
            int page = frame_map[f].page;
            PageEntry *e = &shared_page_tables[proc][page];
            
            // Determine page class
            int class;
            if (!e->referenced && !e->modified) class = 0;
            else if (!e->referenced && e->modified) class = 1;
            else if (e->referenced && !e->modified) class = 2;
            else class = 3;
            
            // Add to corresponding class
            classes[class][counts[class]] = f;
            counts[class]++;
        }
    }
    
    // Find the lowest non-empty class
    int selected_class = -1;
    for (int c = 0; c < 4; c++) {
        if (counts[c] > 0) {
            selected_class = c;
            break;
        }
    }
    
    // If no class is selected, it means all pages are in use and none are classified
    // This can happen if all pages are referenced and modified, which is unlikely in practice
    if (selected_class == -1) {
        // Shouldn't happen - memory is empty
        *out_p = -1;
        *out_pg = -1;
        return;
    }
    
    // Randomly select a page from the chosen class
    int random_index = rand() % counts[selected_class];
    int selected_frame = classes[selected_class][random_index];
    
    *out_p = frame_map[selected_frame].proc;
    *out_pg = frame_map[selected_frame].page;
    
    // Reset the referenced bit for all pages every 10 calls
    // This simulates the periodic reset of the R bit in NRU
    call_counter++;
    if (call_counter % 10 == 0) {
        printf("[NRU] Resetting R bits (call %d)\n", call_counter);
        for (int f = 0; f < NUM_FRAMES; f++) {
            if (frame_map[f].proc >= 0) {
                int proc = frame_map[f].proc;
                int page = frame_map[f].page;
                shared_page_tables[proc][page].referenced = false;
            }
        }
    }
    
    printf("[NRU] Selected P%d page %d from class %d\n", 
           *out_p + 1, *out_pg, selected_class);

}

// Function to select a page using the Second Chance algorithm
// It gives a second chance to pages that have been referenced
void select_2nCh(int *out_p, int *out_pg) {
    while (true) {
        FrameMap fm = frame_map[clock_hand];
        // check if the current frame is in use
        if (fm.proc >= 0) {
            PageEntry *e = &shared_page_tables[fm.proc][fm.page]; // get the page entry
            // if the page has not been referenced, it is a candidate for replacement
            if (!e->referenced) {
                *out_p  = fm.proc;
                *out_pg = fm.page;
                clock_hand = (clock_hand + 1) % NUM_FRAMES;
                return;
            }
            // if the page has been referenced, give it a second chance
            e->referenced = false;
        }
        // avança ponteiro
        clock_hand = (clock_hand + 1) % NUM_FRAMES;
    }
}

// Function to select a page using the LRU (Least Recently Used) algorithm
// It uses an aging counter to keep track of the least recently used pages
void select_LRU(int *out_p, int *out_pg){
    int proc = *out_p; // process that caused the page fault (already passed by GMV)
    int victim_page = -1; // page to be replaced
    unsigned char min_age = 255; // maximum aging counter value (8 bits)

    printf("[LRU] Starting aging update for process P%d...\n", proc + 1);

    for (int pg = 0; pg < NUM_PAGES; pg++) {
        PageEntry *entry = &shared_page_tables[proc][pg];

        if (entry->present) {

            // Select the page with the lowest aging counter
            if (entry->aging_counter < min_age) {
                min_age = entry->aging_counter;
                victim_page = pg;
            }
        }
    }

    if (victim_page == -1) {
        // Fallback: no valid page found (should not happen)
        fprintf(stderr, "[LRU] No present page found for process %d to replace!\n", proc + 1);
        *out_pg = 0;
    } else {
        printf("[LRU] Page %d from process P%d selected for replacement (aging = %d)\n", 
               victim_page, proc + 1, min_age);
        *out_pg = victim_page;
    }

    *out_p = proc; // confirm that the page belongs to the current process
}

// Function to select a page using the Working Set algorithm
// It selects the oldest page that has not been accessed in the last k time units
void select_WS(int *out_p, int *out_pg, int k) {
    int proc = *out_p;
    int victim_page = -1;
    unsigned long oldest_time = ULONG_MAX; // maximum value for unsigned long

    printf("[WS] Working Set: k = %d for process P%d\n", k, proc+1);

    for (int pg = 0; pg < NUM_PAGES; pg++) { // iterate through all pages
        // check if the page is present in RAM
        PageEntry *entry = &shared_page_tables[proc][pg];
        if (entry->present
            && entry->last_access <= clock_counter - k) {
            if (entry->last_access < oldest_time) {
                oldest_time  = entry->last_access;
                victim_page = pg;
            }
        }
    }

    if (victim_page == -1) { // no page found that meets the criteria
        for (int pg = 0; pg < NUM_PAGES; pg++) {
            PageEntry *entry = &shared_page_tables[proc][pg];
            if (entry->present
                && entry->last_access < oldest_time) {
                oldest_time  = entry->last_access;
                victim_page = pg;
            }
        }
    }

    *out_p  = proc;
    *out_pg = victim_page;

    printf("[WS] Selected page %d from process P%d (last_access = %lu)\n",
           victim_page, proc+1, oldest_time);
}

// initializing all frames as free
void initialize_memory() {
    printf("Initializing memory\n");
    for (int i = 0; i < NUM_FRAMES; i++) {
        free_frames[i] = true; // all frames are free for now 
        frame_map[i].proc = -1;
        frame_map[i].page = -1;
    } 
    for (int p = 0; p < NUM_PROCESSES; p++) {
        printf("Initializing TP for process P%d\n", p + 1);
        for (int i = 0; i < NUM_PAGES; i++) {
            shared_page_tables[p][i].frame = -1; // not in any frame
            shared_page_tables[p][i].present = false; // is not present in RAM
            shared_page_tables[p][i].modified = false; // was not modified
            shared_page_tables[p][i].referenced = false; // was not referenced
            shared_page_tables[p][i].aging_counter = 0; // aging counter starts at 0
        }
    }
}

// GMV_Process function is the main process that handles page faults and memory management
// It reads from the pipes of each process, updates the page tables, and handles page faults
void GMV_Process() {
    signal(SIGTERM, term_handler); // handle termination signal
    int i;

    // attaching the shared memory for page tables and page fault flags
    shared_page_tables = (PageEntry (*)[NUM_PAGES]) shmat(shmid, NULL, 0);

    // checking if the shared memory was attached successfully
    if (shared_page_tables == (void *) -1) {
        perror("GMV shmat error");
        exit(EXIT_FAILURE);
    }
    
    // attaching the shared memory for page fault flags
    had_page_fault = shmat(shm_pf, NULL, 0);
    if (had_page_fault == (void *)-1) {
        perror("GMV shmat had_page_fault");
        exit(EXIT_FAILURE);
    }

    // setting all pipes to non-blocking mode
    for (int i = 0; i < NUM_PROCESSES; i++) {
        close(pipes[i][PIPE_WRITE]);  // GMV will just read
        int fl = fcntl(pipes[i][PIPE_READ], F_GETFL);
        fcntl(pipes[i][PIPE_READ], F_SETFL, fl | O_NONBLOCK);
    }

    while (keep_running) { // main loop of GMV process
        for (i = 0; i < NUM_PROCESSES; i++) { // iterating through each process
            ProcessAccess acesso;
            ssize_t bytes = read(pipes[i][PIPE_READ], &acesso, sizeof(ProcessAccess));

            if (bytes == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // no data available, continue to next process
                    continue;
                } else {
                    perror("[GMV] ERROR reading pipe");
                    continue;
                }
            }
            if (bytes == 0) { // no data read, process might have finished
                continue;
            }
            
            printf("[GMV] received from P%d: page = %d, type of acess = %c\n", i+1, acesso.page_number, acesso.access_type);

            PageEntry *entry = &shared_page_tables[i][acesso.page_number];

            // update acess flags
            entry->referenced = true;
            entry->last_access = clock_counter;

            if (acesso.access_type == 'W') entry->modified = true; // if it's a write, mark it as modified

            if (!entry->present) { // page fault
                printf("[GMV] PAGE FAULT in process %d, page %d\n", i+1, acesso.page_number);
                page_faults[i]++; 
                had_page_fault[i] = true;

                // try to find a free frame
                int f = find_free_frame();
                if (f >= 0) {
                    // allocate without subs
                    allocate_frame(i, acesso.page_number, f, acesso.access_type);
                    printf("[GMV] Allocated [P%d] p%d -> frame %d\n", i+1, acesso.page_number, f);
                } else { // necessary subs
                    int victim_process = i;
                    int victim_page;
                    switch (current_Algorithm) {
                        case NRU:
                            select_NRU(&victim_process, &victim_page); 
                            break;
                        case SEGUNDA_CHANCE:
                            select_2nCh(&victim_process, &victim_page);
                            break;
                        case LRU:
                            select_LRU(&victim_process, &victim_page);
                            break;
                        case WORKING_SET:
                            select_WS(&victim_process, &victim_page, k);
                            break;
                        default:
                            printf("[GMV] Invalid.\n");
                            break;
                    }

                    PageEntry *victim_entry = &shared_page_tables[victim_process][victim_page];
                    int freed = victim_entry->frame;
                    if (victim_entry->modified) {
                            printf("[GMV] Writing dirty page %d (P%d) to swap\n", 
                                    victim_page, victim_process+1);
                            swap_writes++;
                    }
                    victim_entry->present = false;
                    free_frames[freed] = true;
                    frame_map[freed].proc = -1;
                    frame_map[freed].page = -1;

                    allocate_frame(i, acesso.page_number, freed, acesso.access_type);
                    printf("[GMV] Allocating page %d from P%d in frame %d\n", acesso.page_number, i+1, freed);
                    printf("[GMV] Substituting [P%d] p%d by [P%d] p%d in frame %d\n",
                           victim_process+1, victim_page,
                           i+1, acesso.page_number, freed);
                }

            }
            else {
                printf("[GMV] [P%d] p%d already in RAM on frame %d\n", i+1, acesso.page_number, entry->frame);
            }
        }

        usleep(QUANTUM);

    }

    for (int i = 0; i < NUM_PROCESSES; i++) kill(processes[i], SIGKILL);
    
    print_summary();
    //printf("here i am");

    // releasing shared memory 
    shmdt(shared_page_tables);
    shmctl(shmid, IPC_RMID, NULL);

    exit(EXIT_SUCCESS);
}

int find_free_frame(){ // finds a free frame in the memory
    // iterates through the free_frames array to find a free frame
    for (int f = 0; f < NUM_FRAMES; f++)
        if (free_frames[f]) return f;
    return -1;
}

void allocate_frame(int proc, int page, int frame, char access_type) { // allocates a frame for a process and page
    free_frames[frame] = false;
    frame_map[frame].proc = proc;
    frame_map[frame].page = page;
    PageEntry *e = &shared_page_tables[proc][page];
    e->frame      = frame;
    e->present    = true;
    e->referenced= true;
    e->modified   = (access_type == 'W');
    e->aging_counter = 0x80;
    e->last_access   = clock_counter;
}

// prints the summary of the GMV simulation
void print_summary() { 
    const char *alg_name;
    switch (current_Algorithm) {
        case NRU:            alg_name = "NRU"; break;
        case SEGUNDA_CHANCE: alg_name = "2nd Chance"; break;
        case LRU:            alg_name = "LRU"; break;
        case WORKING_SET:    alg_name = "Working Set";  break;
        default:             alg_name = "UNKNOWN"; break;
    }

    if (current_Algorithm == WORKING_SET)
        printf("\n=== GMV SIMULATION SUMMARY: %s (k=%d) ===\n\n", alg_name, k);
    else
        printf("\n=== GMV SIMULATION SUMMARY: %s ===\n\n", alg_name);

    printf("Total rounds executed: %d\n\n", num_rounds);

    printf("Page-faults per process:\n");
    for (int p = 0; p < NUM_PROCESSES; p++)
        printf("  P%d: %d\n", p + 1, page_faults[p]);

    printf("\nTotal dirty writes to swap: %d\n\n", swap_writes);

    for (int p = 0; p < NUM_PROCESSES; p++) {
        printf("\n-- Page Table for P%d --\n", p + 1);
        printf("pg  frame  P R M  age\n");
        for (int pg = 0; pg < NUM_PAGES; pg++) {
            PageEntry *e = &shared_page_tables[p][pg];
            char frame_buf[6];                     /* espaço p/ “––” ou “  15”   */
            if (e->present)
                sprintf(frame_buf, "%2d", e->frame);   /* número do quadro */
            else
                strcpy(frame_buf, "--");               /* página não residente */
            char age_buf[4];                      /* " 99", "128" ou "-- "        */
            if (e->present)
                sprintf(age_buf, "%3u", e->aging_counter);
            else
                strcpy(age_buf, "-- ");

            printf("%02d  %5s  %d %d %d  %3s\n",
                pg,             /* índice da página            */
                frame_buf,      /* quadro ou "--"              */
                e->present,     /* bit P                       */
                e->referenced,  /* bit R                       */
                e->modified,    /* bit M                       */
                age_buf);       /* contador ou "--"            */
        }
    }


}

// Signal handler to terminate the GMV process gracefully
// It sets the keep_running flag to 0, which will stop the GMV process loop
static void term_handler(int sig) {
    keep_running = 0;
}

// Updates the aging counters for all pages in all processes
// This simulates the aging mechanism for LRU algorithm
static inline void update_aging_counters(void) {
    for (int p = 0; p < NUM_PROCESSES; p++) {
        for (int pg = 0; pg < NUM_PAGES; pg++) {
            PageEntry *e = &shared_page_tables[p][pg];
            if (!e->present) continue;
            e->aging_counter >>= 1;
            if (e->referenced)
                e->aging_counter |= 0x80;

            e->referenced = false; // reset the referenced bit after aging  
        }
    }
}