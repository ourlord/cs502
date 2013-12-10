/************************************************************************

        This code forms the base of the operating system you will
        build.  It has only the barest rudiments of what you will
        eventually construct; yet it contains the interfaces that
        allow test.c and z502.c to be successfully built together.

        Revision History:
        1.0 August 1990
        1.1 December 1990: Portability attempted.
        1.3 July     1992: More Portability enhancements.
                           Add call to sample_code.
        1.4 December 1992: Limit (temporarily) printout in
                           interrupt handler.  More portability.
        2.0 January  2000: A number of small changes.
        2.1 May      2001: Bug fixes and clear STAT_VECTOR
        2.2 July     2002: Make code appropriate for undergrads.
                           Default program start is in test0.
        3.0 August   2004: Modified to support memory mapped IO
        3.1 August   2004: hardware interrupt runs on separate thread
        3.11 August  2004: Support for OS level locking
		4.0  July    2013: Major portions rewritten to support multiple threads

		Modified by:
			Huanglin, Xiong ( hxiong@wpi.edu )
		5.0 October	 2013: Most of time run in Windows 7, editted, compiled and tested in VS 2010.
						   Done compiling and running on Mingw in Win7 environment, and it works.
						   Also did run in Linux, compiled and tested with gcc on CCC machine 
						   after the last modification.
						   However, more bugs are found when run in Linux than in Windows.
						   Everything personally added or modified is well commented.
************************************************************************/

#include             "global.h"
#include             "syscalls.h"
#include             "protos.h"
#include             "string.h"
#include			<stdlib.h>        //add


// These loacations are global and define information about the page table
extern UINT16        *Z502_PAGE_TBL_ADDR;
extern INT16         Z502_PAGE_TBL_LENGTH;

extern void          *TO_VECTOR [];

char                 *call_names[] = { "mem_read ", "mem_write",
                            "read_mod ", "get_time ", "sleep    ",
                            "get_pid  ", "create   ", "term_proc",
                            "suspend  ", "resume   ", "ch_prior ",
                            "send     ", "receive  ", "disk_read",
                            "disk_wrt ", "def_sh_ar" };

/*==============================   personal add variables   ===============================*/
//definition of error return codes
#define			ERR_ILLEGAL_PRIORITY					31L
#define			ERR_INVALID_NAME						32L
#define			ERR_INVALID_PID							33L
#define			ERR_EXCEED_MAX_PNO						34L
#define			ERR_SUSPEND_SELF						35L
#define			ERR_CONFLICT_SUSPEND					36L
#define			ERR_RESUME_SELF							37L
#define			ERR_ILLEGAL_OPERATION					70L
#define			ERR_ILLEGAL_MESSAGE_LENGTH				38L
#define			ERR_EXCEED_MAX_MSG_NO					39L
#define			ERR_SPECIAL_FAULT_DOING_SUSPEND			502L	//debug use only

#define			MAX_PROCESS_NUMBER						29
#define			MAX_PROCESS_ID							99
#define			MAX_LEGAL_MESSAGE_LENGTH				64
#define			MAX_MESSAGE_NUMBER						10

/*=============================   definition of READ_MODIFY()   ===========================*/
#define			DO_LOCK									1
#define			DO_UNLOCK								0
#define			SUSPEND_UNTIL_LOCKED					TRUE
#define			DO_NOT_SUSPEND							FALSE

#define			TIMER_QUEUE_MEMORY						MEMORY_INTERLOCK_BASE
#define			READY_QUEUE_MEMORY						MEMORY_INTERLOCK_BASE+1
#define			SUSPEND_QUEUE_MEMORY					MEMORY_INTERLOCK_BASE+2
#define			IS_SUSPEND_MEMORY						MEMORY_INTERLOCK_BASE+3
#define			SP_PRINTER_MEMORY						MEMORY_INTERLOCK_BASE+4
#define			DISK_MEMORY								MEMORY_INTERLOCK_BASE+5

/*============================== definition of DISK_READ and DISK_WRITE =====================*/
#define			DO_NOTHING								-1
#define			DO_DISK_READ							0
#define			DO_DISK_WRITE							1

/*============================== definition of Z502_PAGE_TBL_ADDR ============================*/
#define			PTBL_IN_DISK							0x1000

//structure of PCB
typedef struct {
	void *context;
	int process_id;
	int priority;
	char *process_name;
	long wake_up_time;
	BOOL wait_msg;		//flag to see if waiting for a msg
	int disk_id;		//store disk id used by this PCB
	short operation;	//use definition of DISK_READ and DISK_WRITE
	INT32 sector;		//store disk operation information here
	char *data;			//the sector and the data will use in disk operation
}PCB;

//structure of queue
typedef struct TQ {
	PCB *data;
	struct TQ *next;
}*TimerQueue;

//structure of message block
typedef struct {
	int target_pid;
	int source_pid;
	INT32 send_length;
	char message_buffer[MAX_LEGAL_MESSAGE_LENGTH];
	//char *message_buffer;
}MSG;

//structure of message queue
typedef struct MQ {
	MSG *data;
	struct MQ *next;
}*MsgQueue;

//structure of free frame
//when is allocated by memory, this queue will empty
typedef struct FL {
	short frame_number;
	struct FL *next;
}*FrameList;

PCB *current_pcb;			//store current running PCB information
TimerQueue TimerQueue_head;
TimerQueue ReadyQueue_head;
TimerQueue SuspendQueue_head;
MsgQueue MessageQueue_head;
FrameList FrameList_head;

//phase1 global variables
int pid;				//generate process id <-- actually...hard to explain, understand it as a flag
int pno;				//calculate amount of current existing processes
int msg_counter;		//message buffer counter

/*========================     phase2 global variables     ================================*/
UINT16 *address_storage[10];			//store starting address of that PAGE TBL ADDR use in that process
UINT16 *mem_manager[PHYS_MEM_PGS];		//store PAGE TBL ADDR in the page
UINT16 pid_storage[PHYS_MEM_PGS];		//store process id use that PAGE TBl ADDR in the page, maintain simultaneously with above
int mm_idx;			//index for memory organizer
int ref_idx;		//reference index
int sp_print_counter;	//use for limited print
int mp_print_counter;	//use for limited print

BOOL sp_full_print;					//set if the test case need a schedule printer, full print TRUE, otherwise FALSE
BOOL valid_pid[MAX_PROCESS_ID+1];	//check if a pid is valid, valid TRUE, invalid FALSE
BOOL is_suspend[MAX_PROCESS_ID+1];	//i have to do this because some disaster trouble will happen if i do not check 
									//a pid is suspend or not, which will happen in interrupt_handler
									//if suspend TRUE, otherwise FALSE
BOOL mp_full_print;			//flag for memory printer full print
BOOL sp_limited_print;		//flag for schedule printer full print
BOOL mp_limited_print;		//flag for memory printer full print

extern char MEMORY[];		//to get the data from memory

//Virtual Address for lock:
INT32 TimerQLock;
INT32 ReadyQLock;
INT32 SuspendQLock;
INT32 Is_SLock;		//is_suspend <--seems i have to do this, or i cannot run test1f successfully everytime
INT32 SPLock;		//for sp_print(), especially for the get day of time(test1x). u know.....
INT32 DiskLock;

/*=================================   variables define end here   ===========================*/

int generate_pid();
PCB *make_process(int flag, char *pName);
void add_to_timer_queue(PCB *pcb);
BOOL in_timer_queue(int pid);
PCB *remove_from_timer_queue();
void add_to_ready_queue(PCB *pcb);
PCB *remove_from_ready_queue();
void add_to_suspend_queue(PCB *pcb);
void add_to_suspend_queue_end(PCB *pcb);
PCB *remove_from_suspend_queue(int pid);
void add_to_message_queue(MSG *msg);
void start_timer(INT32 time);
void dispatcher();
void create_process(void *pName, void *process, int priority, INT32 *arg4, INT32 *arg5);
BOOL is_valid_process_name(void *name);
void get_process_id(void *name, INT32 *arg2, INT32 *arg3);
void terminate_process(int pid, INT32 *arg2);
void suspend_process(int pid, INT32 *arg2);
void resume_process(int pid, INT32 *arg2);
void sp_print(char *str, PCB *target_pcb, PCB *new_pcb, PCB *kill_pcb, BOOL os_halt, BOOL run_setup_printer);
void sp_print_assistant(char *str, PCB *target_pcb, PCB *new_pcb, PCB *kill_pcb);
void change_priority(int pid, int new_priority, INT32 *arg3);
void send_message(int target_pid, void *message, INT32 length, INT32 *arg4);
void receive_message(int source_pid, void *message, INT32 receive_length, INT32 *arg4, INT32 *arg5, INT32 *arg6);
void add_to_free_frame_list(short frame_number);
short remove_from_free_frame_list();
void mp_print();
void disk_operation(int did, INT32 sector, char *data, INT32 operation);
PCB *remove_from_suspend_queue_by_did(int did);

/************************************************************************
	GENERATE_PID
		Generate a valid pid in range 0~99. To avoid strange things happen if we 
		create too many process so that the SP_SETUP goes wrong, which I can never
		image. Just in case.
		Input: void
		Output: a valid pid
************************************************************************/
int generate_pid()
{
	int output;
	if (pid > MAX_PROCESS_ID)
		pid = 0;
	while (!valid_pid[pid]) {
		pid++;
		if (pid > MAX_PROCESS_ID)
			pid = 0;
	}
	output = pid;
	pid++;
	valid_pid[output] = FALSE;
	return output;
}

/************************************************************************
	MAKE_PROCESS
		Ask the hardware for a context, create the PCB, then call Z502_MAKE_CONTEXT.
		Input: test case number
		Output: PCB structure
************************************************************************/
PCB *make_process(int flag, char *pName)
{
	PCB *pcb;
	void *next_context;
	
	switch (flag) {
		case 0:
			Z502MakeContext(&next_context, (void *)test0, USER_MODE);
			break;
		case 1:
			Z502MakeContext(&next_context, (void *)test1a, USER_MODE);
			break;
		case 2:
			Z502MakeContext(&next_context, (void *)test1b, USER_MODE);
			break;
		case 3:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1c, USER_MODE);
			break;
		case 4:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1d, USER_MODE);
			break;
		case 5:
			Z502MakeContext(&next_context, (void *)test1e, USER_MODE);
			break;
		case 6:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1f, USER_MODE);
			break;
		case 7:
			Z502MakeContext(&next_context, (void *)test1g, USER_MODE);
			break;
		case 8:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1h, USER_MODE);
			break;
		case 9:
			Z502MakeContext(&next_context, (void *)test1i, USER_MODE);
			break;
		case 10:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1j, USER_MODE);
			break;
		case 11:
			Z502MakeContext(&next_context, (void *)test1k, USER_MODE);
			break;
		case 12:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1l, USER_MODE);
			break;
		case 13:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test1m, USER_MODE);
			break;
		case 20:
			mp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2a, USER_MODE);
			break;
		case 21:
			mp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2b, USER_MODE);
			break;
		case 22:
			sp_full_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2c, USER_MODE);
			break;
		case 23:
			sp_full_print = TRUE;
			sp_limited_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2d, USER_MODE);
			break;
		case 24:
			sp_full_print = TRUE;
			mp_full_print = TRUE;
			sp_limited_print = TRUE;
			mp_limited_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2e, USER_MODE);
			break;
		case 25:
			mp_full_print = TRUE;	
			mp_limited_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2f, USER_MODE);
			break;
		case 26:
			//sp_full_print = TRUE;		//debug
			//mp_full_print = TRUE;		//debug
			Z502MakeContext(&next_context, (void *)test2g, USER_MODE);
			break;
		case 27:
			sp_full_print = TRUE;
			mp_full_print = TRUE;
			sp_limited_print = TRUE;
			mp_limited_print = TRUE;
			Z502MakeContext(&next_context, (void *)test2h, USER_MODE);
			break;
		default:
			printf("Input Wrong so we run test0");
			Z502MakeContext(&next_context, (void *)test0, USER_MODE);
	}

	pcb = (PCB *)malloc(sizeof(PCB));
	pcb->context = next_context;
	pcb->priority = 10;
	pcb->process_name = pName;
	pcb->wait_msg = FALSE;
	pcb->process_id = generate_pid();
	pno++;
	//phase2 add
	pcb->operation = DO_NOTHING;
	pcb->disk_id = -1;

	return pcb;
}

/************************************************************************
	ADD_TO_TIMER_QUEUE
		Add a PCB to timer queue in a proper position.
		Input: PCB structure
		Output: void
*************************************************************************/
void add_to_timer_queue(PCB *pcb)
{
	TimerQueue tmp;
	tmp = (struct TQ *)malloc(sizeof(struct TQ));

	//if timer queue is empty
	if (TimerQueue_head == NULL) {
		tmp->data = pcb;
		tmp->next = NULL;
		TimerQueue_head = tmp;
	}
	//if timer queue is not empty
	else {
		TimerQueue tmp2, tmp3;
		//if current pcb wake up time less than the one in the timer q, attach it to the head of timer q
		if (TimerQueue_head->data->wake_up_time > pcb->wake_up_time) {
			tmp->data = pcb;
			tmp->next = TimerQueue_head;
			TimerQueue_head = tmp;
		}
		else {
			tmp2 = tmp = TimerQueue_head;
			//find until current pcb wake up time is less than one item in timer q
			while (tmp->next != NULL && tmp->data->wake_up_time <= pcb->wake_up_time) {
				tmp2 = tmp;
				tmp = tmp->next;
			}
			tmp3 = (struct TQ *)malloc(sizeof(struct TQ));
			tmp3->data = pcb;
			//no single item in the timer q takes less time, attach to the end
			if (tmp->data->wake_up_time <= pcb->wake_up_time) {
				tmp3->next = NULL;
				tmp->next = tmp3;
			}
			//there is a item take more time than current pcb, insert in front of the item
			else {
				tmp3->next = tmp;
				tmp2->next = tmp3;
			}
		}
	}

}

/************************************************************************
	REMOVE_FROM_TIMER_QUEUE
		Remove a PCB from timer queue. Used by interrupt_handler. If the timer queue is empty already,
		print an error and return NULL.
		Input: void
		Output: PCB structure
*************************************************************************/
PCB *remove_from_timer_queue()
{
	if (TimerQueue_head != NULL) {
		TimerQueue tmp;
		tmp = TimerQueue_head;
		TimerQueue_head = TimerQueue_head->next;
		return tmp->data;
	}
	//timer queue is empty, return a print error
	else {
		printf("Timer queue is empty.\n");
		return NULL;
	}
}

/************************************************************************
	IN_TIMER_QUEUE
		Given a process_id and find if that process is in the timer queue.
		Return TRUE if is in the timer queue. Otherwise FALSE.
		Input: pid
		Output: BOOL(TRUE/FALSE)
*************************************************************************/
BOOL in_timer_queue(int pid)
{
	TimerQueue tmp;

	if (TimerQueue_head != NULL) {
		tmp = TimerQueue_head;
		if (tmp->data->process_id == pid)
			return TRUE;
		while (tmp->next != NULL) {
			tmp = tmp->next;
			if (tmp->data->process_id == pid)
				return TRUE;
		}//while
	}//if timer queue head != NULL
	return FALSE;
}

/************************************************************************
	ADD_TO_READY_QUEUE
		Add a PCB to ready queue in a proper position.
		Input: void
		Output: PCB structure
*************************************************************************/
void add_to_ready_queue(PCB *pcb)
{
	TimerQueue tmp;
	tmp = (struct TQ *)malloc(sizeof(struct TQ));

	//if ready queue is empty
	if (ReadyQueue_head == NULL) {
		tmp->data = pcb;
		tmp->next = NULL;
		ReadyQueue_head = tmp;
	}
	//if ready queue is not empty
	else {
		TimerQueue tmp2, tmp3;
		//check the priority and insert to a proper position
		//if current pcb priority is higher than head of ready queue, insert to the head
		if (ReadyQueue_head->data->priority > pcb->priority) {
			tmp->data = pcb;
			tmp->next = ReadyQueue_head;
			ReadyQueue_head = tmp;
		}
		else {
			tmp2 = tmp = ReadyQueue_head;
			//find a position where priority is lower than current pcb priority
			while (tmp->next != NULL && tmp->data->priority <= pcb->priority) {
				tmp2 = tmp;
				tmp = tmp->next;
			}
			tmp3 = (struct TQ *)malloc(sizeof(struct TQ));
			tmp3->data = pcb;
			//no single item's priority in ready queue is lower than current one, attach to the end
			if (tmp->data->priority <= pcb->priority) {
				tmp3->next = NULL;
				tmp->next = tmp3;
			}
			//there is one item's priority in ready queue is lower than current one, insert in front of the item
			else {
				tmp3->next = tmp;
				tmp2->next = tmp3;
			}
		}
	}
}

/************************************************************************
	REMOVE_FROM_READY_QUEUE
		Remove a PCB from ready queue. Used by terminate_process. If the ready queue is empty already, 
		print an error and return NULL.
		Input: void
		Output: pcb structure
*************************************************************************/
PCB *remove_from_ready_queue()
{
	if (ReadyQueue_head != NULL) {
		TimerQueue tmp;
		tmp = ReadyQueue_head;
		ReadyQueue_head = ReadyQueue_head->next;
		return tmp->data;
	}
	//ready queue is empty return a print error
	else {
		printf("Ready queue is empty.\n");
		return NULL;
	}
}

/************************************************************************
	ADD_TO_SUSPEND_QUEUE
		Add a PCB to suspend queue at its head.
		Input: PCB structure
		Output: PCB void
*************************************************************************/
void add_to_suspend_queue(PCB *pcb)
{
	TimerQueue tmp;
	tmp = (struct TQ *)malloc(sizeof(struct TQ));

	tmp->data = pcb;
	tmp->next = SuspendQueue_head;
	SuspendQueue_head = tmp;
}

/************************************************************************
	ADD_TO_SUSPEND_QUEUE_END
		Add a PCB to suspend queue at its end.
		Input: PCB structure
		Output: void
*************************************************************************/
void add_to_suspend_queue_end(PCB *pcb)
{
	TimerQueue tmp, tmp2;
	tmp = (struct TQ *)malloc(sizeof(struct TQ));

	tmp->data = pcb;
	tmp->next = NULL;
	if (SuspendQueue_head == NULL)
		SuspendQueue_head = tmp;
	else {
		tmp2 = SuspendQueue_head;
		while (tmp2->next != NULL) {
			tmp2 = tmp2->next;
		}
		tmp2->next = tmp;
	}
}

/************************************************************************
	REMOVE_FROM_SUSPEND_QUEUE
		Remove a PCB with given process id from suspend queue. If the suspend queue is empty already, 
		print an error and return NULL.
		Input: pid
		Output: pcb structure
*************************************************************************/
PCB *remove_from_suspend_queue(int pid)
{
	TimerQueue tmp, tmp2;

	if (SuspendQueue_head != NULL) {
		if (SuspendQueue_head->data->process_id == pid) {
			tmp = SuspendQueue_head;
			if (SuspendQueue_head->next == NULL)
				SuspendQueue_head = NULL;
			else
				SuspendQueue_head = SuspendQueue_head->next;
			return tmp->data;
		}
		//not the first item in suspend q need to remove
		else {
			tmp = SuspendQueue_head;
			while (tmp->next != NULL) {
				tmp2 = tmp;
				tmp = tmp->next;
				if (tmp->data->process_id == pid) {
					tmp2->next = tmp->next;
					return tmp->data;
				}
			}//while
			printf("PID is not found in suspend queue.\n");
			return NULL;
		} //else
	}
	//suspend q is empty return a print error(we do not expect this to happen)
	else {
		printf("Suspend Queue is empty.\n");
		return NULL;
	}
}

/************************************************************************
	ADD_TO_MESSAGE_QUEUE
		Add a message block to message queue. 
		Input: message block
		Output: void
*************************************************************************/
void add_to_message_queue(MSG *msg)
{
	MsgQueue tmp, tmp2;
	tmp2 = (struct MQ *)malloc(sizeof(struct MQ));

	tmp2->data = msg;
	tmp2->next = NULL;

	if (MessageQueue_head != NULL) {
		tmp = MessageQueue_head;
		while (tmp->next != NULL)
			tmp = tmp->next;
		tmp->next = tmp2;
	}
	else {
		MessageQueue_head = tmp2;
	}
}

/************************************************************************
	START_TIMER
		The process becomes "not ready to run" for the number of time units given in "sleep_time".
		Input: sleep_time
		Output: void
*************************************************************************/
void start_timer(INT32 time)
{
	INT32 Status, Time;

	//if sleep time is 0, add to ready queue directly
	if (time == 0) {
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		add_to_ready_queue(current_pcb);
		READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
	}
	//otherwise
	else {
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		MEM_READ(Z502ClockStatus, &Time);
		current_pcb->wake_up_time = Time + time;
		MEM_READ(Z502TimerStatus, &Status);
		if (Status == DEVICE_FREE) {
			MEM_WRITE(Z502TimerStart, &time);
		}
		//if timer is working, check if timer q is empty or current sleep time is less than the one settled in timer q
		//reset the timer
		else if (TimerQueue_head == NULL || TimerQueue_head->data->wake_up_time > current_pcb->wake_up_time) {
			MEM_WRITE(Z502TimerStart, &time);
		}
		//if first item in timer q wake up timer is equal or smaller than current one, do not set the timer
		//directly attach current event to timer q
		
		READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
		sp_print("Sleep", current_pcb, NULL, NULL, FALSE, TRUE);
		READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

		add_to_timer_queue(current_pcb);
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
	}//else otherwise
	dispatcher();			//as jb said....we need something to use remove_from_ready_queue(), and dealing
							//with next interrupt event....................
}

/************************************************************************
	DISPATCHER
		When ready queue is empty, call Z502Idle(). Remove the 1st item in ready queue, and swith context.
		Called after the start_timer.
		However, there is some problem about where Z502Idle() should put. Let's see if it can work before 
		while() or just put it in while() as jb said.
		For now, it works well in while(). But if the sleep time is long enough, maybe it will interrupt 
		by Z502Idle() itself(cannot be called repeatly for more than 10 times).
		Input: void
		Output: void
*************************************************************************/
void dispatcher()
{
	if (ReadyQueue_head == NULL)
		CALL(Z502Idle());
	while (ReadyQueue_head == NULL) {
		//CALL(Z502Idle());
		CALL();
	}

	READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
	current_pcb = remove_from_ready_queue();
	
	READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
	sp_print("Dispatch", current_pcb, NULL, NULL, FALSE, TRUE);
	READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

	READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);

	Z502SwitchContext(SWITCH_CONTEXT_SAVE_MODE, &(current_pcb->context));
}

/************************************************************************
	CREATE_PROCESS
		Create a process. This process will have name "process_name", will begin execution at 
		location "starting_address" (in essence, this is the name of a routine), and at the 
		start of execution has a priority of "initial_priority". The system call returns the 
		process_id of the created process. An error will be generated if a process with the same 
		name already exists. Lots of other errors are also possible.
		Success means Error # = 0.
		Input: process_name, starting_address, initial_priority, arg4(return pid), arg5(return Error #)
		Output: void
*************************************************************************/
void create_process(void *pName, void *process, int priority, INT32 *arg4, INT32 *arg5)
{
	*arg4 = -1;
	*arg5 = ERR_SUCCESS;

	//check priority
	if (priority < 0 || priority > 100) {
		*arg5 = ERR_ILLEGAL_PRIORITY;
		return;
	}
	if (!is_valid_process_name(pName)) {
		*arg5 = ERR_INVALID_NAME;
		return;
	}
	else {
		if (pno < MAX_PROCESS_NUMBER) {
			void *next_context;
			PCB *pcb;
			size_t size;
			char *strtmp;

			pcb = (PCB *)malloc(sizeof(PCB));
			Z502MakeContext(&next_context, process, USER_MODE);
			pcb->context = next_context;
			pcb->priority = priority;
			pcb->process_id = generate_pid();
			pno++;
			*arg4 = pcb->process_id;

			//operate the process name
			size = strlen((char *)pName);
			strtmp = (char *)malloc(size+1);
			strcpy(strtmp, (const char*)pName);
			pcb->process_name = strtmp;
			pcb->wait_msg = FALSE;
			//phase2 add
			pcb->operation = DO_NOTHING;
			pcb->disk_id = -1;

			READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			add_to_ready_queue(pcb);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			sp_print("Create", pcb, pcb, NULL, FALSE, TRUE);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		}
		else {
			*arg5 = ERR_EXCEED_MAX_PNO;
		}//else if (pno < max)
	}//else if (check p name)
}

/************************************************************************
	IS_VALID_PROCESS_NAME
		Check if a given process name is valid to use. Valid TRUE, invalid FALSE.
		Input: process name
		Output: BOOL(TRUE/FALSE)
*************************************************************************/
BOOL is_valid_process_name(void *name)
{
	TimerQueue tmp;
	
	if (strcmp("", (const char *)name) == 0) {
		return FALSE;
	}
	//check timer q
	if (TimerQueue_head != NULL) {
		tmp = TimerQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			return FALSE;
		}
		else {
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
					return FALSE;
				}
			}
		}
	}
	//check ready q
	if (ReadyQueue_head != NULL) {
		tmp = ReadyQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			return FALSE;
		}
		else {
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
					return FALSE;
				}
			}
		}
	}
	//check suspend q
	if (SuspendQueue_head != NULL) {
		tmp = SuspendQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			return FALSE;
		}
		else {
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
					return FALSE;
				}
			}
		}
	}
	return TRUE;
}

/************************************************************************
	GET_PROCESS_ID
		Returns the process_id for the process whose name is "process_name". If process_name = "", 
		then return the PID of the calling process. An error is returned if a process with the 
		requested name doesn't exist. Lots of other errors are also possible.
		Success means Error # = 0.
		Input: process_name, arg2(return pid), arg3(return Error #)
		Output: void
*************************************************************************/
void get_process_id(void *name, INT32 *arg2, INT32 *arg3)
{
	TimerQueue tmp;
	*arg2 = -1;
	*arg3 = ERR_SUCCESS;

	//process name is "", get current process id
	if (strcmp("", (const char *)name) == 0) {
		*arg2 = current_pcb->process_id;
		return;
	}
	//check cuurent pcb
	if (strcmp(current_pcb->process_name, (const char *)name) == 0) {
		*arg2 = current_pcb->process_id;
		return;
	}
	//check timer queue
	if (TimerQueue_head != NULL) {
		tmp = TimerQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			*arg2 = tmp->data->process_id;
			return;
		}
		while (tmp->next != NULL) {
			tmp = tmp->next;
			if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
				*arg2 = tmp->data->process_id;
				return;
			}
		}
	}
	//check ready queue
	if (ReadyQueue_head != NULL) {
		tmp = ReadyQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			*arg2 = tmp->data->process_id;
			return;
		}
		while (tmp->next != NULL) {
			tmp = tmp->next;
			if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
				*arg2 = tmp->data->process_id;
				return;
			}
		}
	}
	//check suspend queue
	if (SuspendQueue_head != NULL) {
		tmp = SuspendQueue_head;
		if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
			*arg2 = tmp->data->process_id;
			return;
		}
		while (tmp->next != NULL) {
			tmp = tmp->next;
			if (strcmp(tmp->data->process_name, (const char *)name) == 0) {
				*arg2 = tmp->data->process_id;
				return;
			}
		}
	}
	*arg3 = ERR_INVALID_NAME;
}

/************************************************************************
	TERMINATE_PROCESS
		Terminate the process whose PID is given by "process_id". If process_id = -1, then 
		terminate self. If process_id = -2, then terminate self and any child processes. 
		Termination of a non-existent process results in an error being returned. Upon 
		termination, all information about a process is lost and it will never run again. 
		An error is returned if a process with that PID doesn't exist. Lots of other errors 
		are also possible, for instance, the target process isn't in the hierarchy (isn't a 
		child of) the requester.
		Success means Error # = 0.
		Input: pid, arg2(return Error #)
		Output: void
*************************************************************************/
void terminate_process(int pid, INT32 *arg2)
{
	TimerQueue tmp, tmp2;
	*arg2 = ERR_SUCCESS;
	//pid = -2, halt the os
	//or pid = 0, terminate test case
	if (pid == -2) {
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
		sp_print("OS_Halt", current_pcb, NULL, current_pcb, TRUE, TRUE);
		Z502Halt();
	}
	//pid = -1 kill self
	else if (pid == -1 || pid == current_pcb->process_id) {
		if (ReadyQueue_head == NULL && TimerQueue_head == NULL){
			READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
			READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			sp_print("OS_Halt", current_pcb, NULL, current_pcb, TRUE, TRUE);
			Z502Halt();
		}
		if (ReadyQueue_head == NULL)
			CALL(Z502Idle());
		while (ReadyQueue_head == NULL){
			//CALL(Z502Idle());
			CALL();
		}
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);

		READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
		sp_print("Done", current_pcb, NULL, current_pcb, FALSE, TRUE);
		READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

		valid_pid[current_pcb->process_id] = TRUE;
		current_pcb = remove_from_ready_queue();
		pno--;
		READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);

		Z502SwitchContext(SWITCH_CONTEXT_KILL_MODE, &(current_pcb->context));
	}
	//otherwise
	else {
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		if (ReadyQueue_head != NULL) {
			if (ReadyQueue_head->data->process_id == pid) {
				if (ReadyQueue_head->next == NULL) {
					ReadyQueue_head = NULL;
					valid_pid[pid] = TRUE;	//release pid
					pno--;
				}
				else {
					ReadyQueue_head = ReadyQueue_head->next;
					valid_pid[pid] = TRUE;	//release pid
					pno--;
				}//end ready q next = nul
			}//end if head->pid = pid
			else {
				tmp = ReadyQueue_head;
				while (tmp->next != NULL) {
					tmp2 = tmp;
					tmp = tmp->next;
					if (tmp->data->process_id == pid){
						tmp2->next = tmp->next;
						valid_pid[pid] = TRUE;	//release pid
						pno--;
					}
				}//end while
			}//end else
		}//end if ready q != null
		READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
	}//end otherwise
}

/************************************************************************
	SUSPEND_PROCESS
		Suspend the process whose PID is given. If pid = -1, then suspend self.
		Suspension of an already suspended process results in no action being taken.
		Upon suspension, a process is removed from the ready queue; it will not run 
		again until that process is the target of a RESUME_PROCESS. An error is returned 
		if a process with that PID doesn't exist. Lots of other errors are also possible. 
		Success means error = 0.
		Input: pid, arg2(return Error #)
		Output: void
************************************************************************/
void suspend_process(int pid, INT32 *arg2)
{
	TimerQueue tmp, tmp2;
	*arg2 = ERR_SUCCESS;

	if (pid < -1 || pid > MAX_PROCESS_ID) {
		*arg2 = ERR_INVALID_PID;
		return;
	}
	//suspend self
	if (pid == -1 || pid == current_pcb->process_id) {
		if (!is_suspend[current_pcb->process_id]) {
			//if timer queue is not empty, remove self to suspend queue and do dispatcher
			if (TimerQueue_head != NULL) {
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

				READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				sp_print("Suspend", current_pcb, NULL, NULL, FALSE, TRUE);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

				is_suspend[current_pcb->process_id] = TRUE;
				add_to_suspend_queue(current_pcb);
				READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				dispatcher();
				return;
			}
			//if timer queue is empty, but ready queue is not empty, add self to suspend queue and do dispatcher
			else {
				if (ReadyQueue_head != NULL) {
					READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
					READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

					READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
					sp_print("Suspend", current_pcb, NULL, NULL, FALSE, TRUE);
					READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

					is_suspend[current_pcb->process_id] = TRUE;
					add_to_suspend_queue(current_pcb);
					READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
					READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
					dispatcher();
					return;
				}
				//both timer queue and ready queue are empty, halt the os
				else {
					terminate_process(-2, arg2);
					return;
				}
			}//else(if timer q empty)
		}
		//if self is already suspended
		else if (!current_pcb->wait_msg){
			*arg2 = ERR_CONFLICT_SUSPEND;
			return;
		}
	}//if suspend self
	//suspend other process
	else {
		if (!is_suspend[pid]) {
			if (!in_timer_queue(pid)) {
				READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
				tmp2 = tmp = ReadyQueue_head;
				//find the target
				if (tmp->data->process_id != pid) {
					while (tmp->next != NULL) {
						tmp2 = tmp;
						tmp = tmp->next;
						if (tmp->data->process_id == pid) {
							tmp2->next = tmp->next;
							break;
						}
					}//while
				}
				//if it is the first item in ready needs to suspend
				else {
					ReadyQueue_head = tmp->next;
				}
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

				READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				sp_print("Suspend", tmp->data, NULL, NULL, FALSE, TRUE);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

				add_to_suspend_queue(tmp->data);			
				is_suspend[tmp->data->process_id] = TRUE;
				READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			}
			//if the suspend process is in timer q, we set the flag and operate it later in interrupt handler
			READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
			is_suspend[pid] = TRUE;
			READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
		}
		//pid is already in suspend q
		else {
			tmp = SuspendQueue_head;
			if (tmp->data->process_id == pid && tmp->data->wait_msg) {
				return;
			}
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (tmp->data->process_id == pid && tmp->data->wait_msg)
						return;
			}
			*arg2 = ERR_CONFLICT_SUSPEND;
		}
	}//else
}

/************************************************************************
	RESUME_PROCESS
		Resume the process whose PID is given. Resumption of an not-suspended process results 
		in an error. Upon resumption, a process is again placed on the ready queue and is able 
		to run. An error is returned if a process with that target PID doesn't exist. Lots of 
		other errors are also possible.
		Input: pid, arg2(return Error #)
		Output: void
************************************************************************/
void resume_process(int pid, INT32 *arg2)
{
	PCB *pcb;
	*arg2 = ERR_SUCCESS;

	if (pid == -1 || pid == current_pcb->process_id) {
		*arg2 = ERR_RESUME_SELF;
		return;
	}
	if (pid < 0 || pid >MAX_PROCESS_ID || valid_pid[pid]) {
		*arg2 = ERR_INVALID_PID;
		return;
	}
	if (is_suspend[pid]) {
		if (!in_timer_queue(pid)) {
			READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
			
			pcb = remove_from_suspend_queue(pid);
			is_suspend[pcb->process_id] = FALSE;

			READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			sp_print("Resume", pcb, NULL, NULL, FALSE, TRUE);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);

			add_to_ready_queue(pcb);

			READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			
		}
		//if the resume process is in timer q, we set the flag and operate it later in interrupt handler
		READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
		is_suspend[pid] = FALSE;
		READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
	}
	//if the pid is not in suspend q
	else {
		TimerQueue tmp;
		tmp = ReadyQueue_head;
		if (tmp->data->process_id == pid && tmp->data->wait_msg) {
			return;
		}
		else {
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (tmp->data->process_id == pid && tmp->data->wait_msg) {
					return;
				}
			}
		}
		*arg2 = ERR_ILLEGAL_OPERATION;
	}
	
}

/************************************************************************
	SP_PRINT
		Call schedule printer to print something.
		Input: action mode, target mode pcb, new mode pcb, if terminate mode use, if run setup_print
		Output: void
************************************************************************/
void sp_print(char *str, PCB *target_pcb, PCB *new_pcb, PCB *kill_pcb, BOOL os_halt, BOOL run_setup_printer)
{
	if (sp_full_print) {
		//if limited print is set-up and it already print so many things that we want to show, do nothing then
		if (sp_limited_print && sp_print_counter++ > 10) {
			return;
		}
		//else we nornally setup the printer and print
		else {
			if (run_setup_printer) {
				TimerQueue tmp;
				//timer queue
				if (TimerQueue_head != NULL) {
					tmp = TimerQueue_head;
					CALL(SP_setup(SP_WAITING_MODE, tmp->data->process_id));
					while (tmp->next != NULL) {
						tmp = tmp->next;
						CALL(SP_setup(SP_WAITING_MODE, tmp->data->process_id));
					}
				}
				//ready queue
				if (ReadyQueue_head != NULL) {
					tmp = ReadyQueue_head;
					CALL(SP_setup(SP_READY_MODE, tmp->data->process_id));
					while (tmp->next != NULL) {
						tmp = tmp->next;
						CALL(SP_setup(SP_READY_MODE, tmp->data->process_id));
					}
				}
				//suspend queue
				if (SuspendQueue_head != NULL) {
					tmp = SuspendQueue_head;
					CALL(SP_setup(SP_SUSPENDED_MODE, tmp->data->process_id));
					while (tmp->next != NULL) {
						tmp = tmp->next;
						CALL(SP_setup(SP_SUSPENDED_MODE, tmp->data->process_id));
					}
				}
			}//if run_setup_printer
			if (os_halt) {
				TimerQueue tmp;
				if (TimerQueue_head != NULL) {
					tmp = TimerQueue_head;
					sp_print_assistant(str, tmp->data, NULL, tmp->data);
					while (tmp->next != NULL) {
						tmp = tmp->next;
						sp_print_assistant(str, tmp->data, NULL, tmp->data);
					}
				}//if timer q
				if (ReadyQueue_head != NULL) {
					tmp = ReadyQueue_head;
					sp_print_assistant(str, tmp->data, NULL, tmp->data);
					while (tmp->next != NULL) {
						tmp = tmp->next;
						sp_print_assistant(str, tmp->data, NULL, tmp->data);
					}
				}// if ready q
				if (SuspendQueue_head != NULL) {
					tmp = SuspendQueue_head;
					sp_print_assistant(str, tmp->data, NULL, tmp->data);
					while (tmp->next != NULL) {
						tmp = tmp->next;
						sp_print_assistant(str, tmp->data, NULL, tmp->data);
					}
				}//if suspend q
			}//if os_halt
			sp_print_assistant(str, target_pcb, new_pcb, kill_pcb);
		}
	}//if sp_full_print
}

/************************************************************************
	SP_PRINT_ASSISTANT
		A simple method helps sp_print().
		Input: action mode, target mode pcb, new mode pcb
		Output: void
************************************************************************/
void sp_print_assistant(char *str, PCB *target_pcb, PCB *new_pcb, PCB *kill_pcb)
{
	CALL(SP_setup(SP_TARGET_MODE, target_pcb->process_id));
	if (new_pcb != NULL) 
		CALL(SP_setup(SP_NEW_MODE, new_pcb->process_id));
	CALL(SP_setup(SP_RUNNING_MODE, current_pcb->process_id));
	if (kill_pcb != NULL)
		CALL(SP_setup(SP_TERMINATED_MODE, kill_pcb->process_id));
		CALL(SP_setup_action(SP_ACTION_MODE, str));
		CALL(SP_print_header());
		CALL(SP_print_line());
}

/************************************************************************
	CHANGE_PRIORITY
		Change the priority of the process whose PID is given by "process_id". If 
		process_id = -1, then change self. The result of a change priority takes 
		effect immediately. An error is returned if a process with that target PID 
		doesn't exist. Lots of other errors are also possible.
		Success means Error # = 0
		Input: pid, new priority, arg3(return Error #)
		Output: void
************************************************************************/
void change_priority(int pid, int new_priority, INT32 *arg3)
{
	TimerQueue tmp, tmp2;
	*arg3 = ERR_SUCCESS;

	if (pid < -1 || pid > MAX_PROCESS_ID || valid_pid[pid]) {
		*arg3 = ERR_INVALID_PID;
		return;
	}
	if (new_priority < 0 || new_priority > 100) {
		*arg3 = ERR_ILLEGAL_PRIORITY;
		return;
	}
	//pid = -1 operate current pcb, change it directly
	if (pid == -1 || pid == current_pcb->process_id) {
		current_pcb->priority = new_priority;
	}
	//other situation
	else {
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		//check timer q
		if (TimerQueue_head != NULL) {
			tmp = TimerQueue_head;
			if (tmp->data->process_id == pid) {
				printf("Priority Change: PID = %d in TimerQueue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
				tmp->data->priority = new_priority;
				READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				READ_MODIFY(TIMER_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
				return;
			}
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (tmp->data->process_id == pid) {
					printf("Priority Change: PID = %d in TimerQueue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
					tmp->data->priority = new_priority;
					READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
					sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
					READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
					READ_MODIFY(TIMER_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
					return;
				}
			}//while
		}//timer q operate end
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		//check suspend q
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		if (SuspendQueue_head != NULL) {
			tmp = SuspendQueue_head;
			if (tmp->data->process_id == pid) {
				printf("Priority Change: PID = %d in SuspendQueue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
				tmp->data->priority = new_priority;
				READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				return;
			}
			while (tmp->next != NULL) {
				tmp = tmp->next;
				if (tmp->data->process_id == pid) {
					printf("Priority Change: PID = %d in SuspendQueue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
					tmp->data->priority = new_priority;
					READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
					sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
					READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
					READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
					return;
				}
			}//while
		}//suspend q operate end
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		//check ready q, this is complicated...
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		if (ReadyQueue_head != NULL) {
			tmp = ReadyQueue_head;
			if (tmp->data->process_id == pid) {
				printf("Priority Change: PID = %d in Ready Queue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
				tmp->data->priority = new_priority;
				ReadyQueue_head = tmp->next;
				add_to_ready_queue(tmp->data);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
				sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
				READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			}//we dont need a return here because the program will only go one of these routes
			//first item in ready q does not match, find the rest
			else {
				while (tmp->next != NULL) {
					tmp2 = tmp;
					tmp = tmp->next;
					if (tmp->data->process_id == pid) {
						printf("Priority Change: PID = %d in ReadyQueue, Priority: %d --> %d\n", tmp->data->process_id, tmp->data->priority, new_priority);
						tmp->data->priority = new_priority;
						tmp2->next = tmp->next;
						add_to_ready_queue(tmp->data);
						READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
						sp_print("Priority", tmp->data, NULL, NULL, FALSE, TRUE);
						READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
						READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
						return;
					}
				}//while
			}//else
		}//ready q operate end
		READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
	}//legal situation
}

/************************************************************************
    SEND_MESSAGE
        Send a message to the target process. When the target does a "RECEIVE_MESSAGE", place 
		data from this send in the message buffer of that target. message_send_length is the 
		size of the send buffer - it must be larger than or equal in size to the string of data 
		that is actually sent; so this parameter is a buffer size rather than a message size. If 
		the target_pid = -1, then broadcast the message to all potential receivers (although this 
		message will actually be intercepted by only one of those receivers).
		Success means Error # = 0.
		Input: target_pid, message, message_length, arg4(return Error #)
		Output: void
************************************************************************/
void send_message(int target_pid, void *message, INT32 length, INT32 *arg4)
{
	TimerQueue tmp;
	size_t size;
	MSG *msg;
	*arg4 = ERR_SUCCESS;

	if (target_pid < -1 || target_pid > MAX_PROCESS_ID) {
		*arg4 = ERR_INVALID_PID;
		return;
	}
	if (length <= 0 || length > MAX_LEGAL_MESSAGE_LENGTH) {
		*arg4 = ERR_ILLEGAL_MESSAGE_LENGTH;
		return;
	}
	//if a broadcast
	if (target_pid == -1) {
		//find suspend queue to see if there is an item waiting to receive message
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		if (SuspendQueue_head != NULL) {
			tmp = SuspendQueue_head;
			if (tmp->data->wait_msg) {
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				resume_process(tmp->data->process_id, arg4);
			}//if an item in suspend queue is waiting for message
			else {
				while (tmp->next != NULL) {
					tmp = tmp->next;
					if (tmp->data->wait_msg) {
						READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
						resume_process(tmp->data->process_id, arg4);
						break;
					}
				}//while
			}//else
		}
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
	}
	//otherwise
	else {
		//find the target pid is in suspend queue
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		if (SuspendQueue_head != NULL) {
			tmp = SuspendQueue_head;
			if (tmp->data->process_id == target_pid && tmp->data->wait_msg) {
				READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
				resume_process(tmp->data->process_id, arg4);
			}
			else {
				while (tmp->next != NULL) {
					tmp = tmp->next;
					if (tmp->data->wait_msg && tmp->data->process_id == target_pid) {
						READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
						resume_process(tmp->data->process_id, arg4);
						break;
					}
				}//while
			}//else
		}//if suspend is not empty
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
	}
	if (++msg_counter <= MAX_MESSAGE_NUMBER) {
		msg = (MSG *)malloc(sizeof(MSG));
		msg->send_length = length;
		msg->target_pid = target_pid;
		msg->source_pid = current_pcb->process_id;
		size = strlen((const char *)message);
		strncpy(msg->message_buffer, (const char *)message, size+1);
		//printf("send:sender: %d, receiver: %d, length: %d\n", msg->source_pid, msg->target_pid, msg->send_length);
		//printf("send:msg=%s msg->msg_b=%s\n", message, msg->message_buffer);
		add_to_message_queue(msg);
	}
	else {
		*arg4 = ERR_EXCEED_MAX_MSG_NO;
		return;
	}
}

/************************************************************************
    RECEIVE_MESSAGE
        Receive a message from source_pid. If source_pid = -1, then receive from any sender who has 
		specifically targeted you or who has done a broadcast. If -1 is used, then &message_sender_pid 
		contains the pid of the process that did the send. The Operating System will place that message 
		in message buffer if it is less than message_receive_length bytes long. message_receive_length 
		is the size available in the receive buffer. Return the size of the send buffer in 
		message_send_length. In general, a RECEIVE_MESSAGE system call causes the receiver process to 
		suspend itself until the SEND is made. Devise appropriate rules of behaviour for the sender and 
		receiver.
		Success means Error # = 0.
		Input: source_pid, message, receive_length, arg4(return send length), arg5(return sender pid), arg6(return Error #)
		Output: void
************************************************************************/
void receive_message(int source_pid, void *message, INT32 receive_length, INT32 *arg4, INT32 *arg5, INT32 *arg6)
{
	MsgQueue tmp, tmp2;
	size_t size;
	*arg6 = ERR_SUCCESS;

	if (source_pid < -1 || source_pid > MAX_PROCESS_ID) {
		*arg6 = ERR_INVALID_PID;
		return;
	}
	if (receive_length <=0 || receive_length > MAX_LEGAL_MESSAGE_LENGTH) {
		*arg6 = ERR_ILLEGAL_MESSAGE_LENGTH;
		return;
	}
	if (MessageQueue_head != NULL) {
		tmp = MessageQueue_head;
		//if broadcast
		if (source_pid == -1) {
			//message q head is the one we want
			if (tmp->data->target_pid == -1 || tmp->data->target_pid == current_pcb->process_id) {
				if (tmp->data->send_length <= receive_length) {
					*arg4 = tmp->data->send_length;
					*arg5 = tmp->data->source_pid;
					size = strlen(tmp->data->message_buffer);
					strncpy((char *)message, tmp->data->message_buffer, size+1);
					//printf("sender: %d, receiver: %d, length: %d\n", tmp->data->source_pid, tmp->data->target_pid, tmp->data->send_length);
					//printf("msg=%s msg->msg_b=%s\n", message, tmp->data->message_buffer);
					current_pcb->wait_msg = FALSE;
					MessageQueue_head = MessageQueue_head->next;
					msg_counter--;
					return;
				}
				//if the length is illegal
				else {
					*arg6 = ERR_ILLEGAL_MESSAGE_LENGTH;
					return;
				}
			}
			else {
				while (tmp->next != NULL) {
					tmp2 = tmp;
					tmp = tmp->next;
					if (tmp->data->target_pid == -1 || tmp->data->target_pid == current_pcb->process_id) {
						if (tmp->data->send_length <= receive_length) {
							*arg4 = tmp->data->send_length;
							*arg5 = tmp->data->source_pid;
							size = strlen(tmp->data->message_buffer);
							strncpy((char *)message, tmp->data->message_buffer, size+1);
							//printf("sender: %d, receiver: %d, length: %d\n", tmp->data->source_pid, tmp->data->target_pid, tmp->data->send_length);
							//printf("msg=%s msg->msg_b=%s\n", message, tmp->data->message_buffer);
							current_pcb->wait_msg = FALSE;
							tmp2->next = tmp->next;
							msg_counter--;
							return;
						}
						else {
							*arg6 = ERR_ILLEGAL_MESSAGE_LENGTH;
							return;
						}
					}
				}//while
			}
		}
		//source pid != -1
		else {
			if ((tmp->data->source_pid == source_pid) && (tmp->data->target_pid == current_pcb->process_id)) {
				if (tmp->data->send_length <= receive_length) {
					*arg4 = tmp->data->send_length;
					*arg5 = tmp->data->source_pid;
					size = strlen(tmp->data->message_buffer);
					strncpy((char *)message, tmp->data->message_buffer, size+1);
					//printf("sender: %d, receiver: %d, length: %d\n", tmp->data->source_pid, tmp->data->target_pid, tmp->data->send_length);
					//printf("msg=%s msg->msg_b=%s\n", message, tmp->data->message_buffer);
					current_pcb->wait_msg = FALSE;
					MessageQueue_head = MessageQueue_head->next;
					msg_counter--;
					return;
				}
				else {
					*arg6 = ERR_ILLEGAL_MESSAGE_LENGTH;
					return;
				}
			}
			else {
				while (tmp->next != NULL) {
					tmp2 = tmp;
					tmp = tmp->next;
					if ((tmp->data->source_pid == source_pid) && (tmp->data->target_pid == current_pcb->process_id)) {
						if (tmp->data->send_length <= receive_length) {
							*arg4 = tmp->data->send_length;
							*arg5 = tmp->data->source_pid;
							size = strlen(tmp->data->message_buffer);
							strncpy((char *)message, tmp->data->message_buffer, size+1);
							//printf("sender: %d, receiver: %d, length: %d\n", tmp->data->source_pid, tmp->data->target_pid, tmp->data->send_length);
							//printf("msg=%s msg->msg_b=%s\n", message, tmp->data->message_buffer);
							current_pcb->wait_msg = FALSE;
							tmp2->next = tmp->next;
							msg_counter--;
							return;
						}
						else {
							*arg6 = ERR_ILLEGAL_MESSAGE_LENGTH;
							return;
						}
					}
				}//while
			}//else(not the first item to operate)
		}//else(not a broadcast)
	}//if message q is not empty
	//message queue is empty or we cannot find what we want in message queue
	current_pcb->wait_msg = TRUE;
	suspend_process(-1, arg6);
	//after resume, we do receive again
	receive_message(source_pid, message, receive_length, arg4, arg5, arg6);
}

/************************************************************************
	ADD_TO_FREE_FRAME_LIST
		Construct a free frame list called in os_init. We maintain this list to see if our system 
		have a free frame later if need.
		Input: frame_number
		Output: 
*************************************************************************/
void add_to_free_frame_list(short frame_number)
{
	FrameList tmp, tmp2;
	tmp = (struct FL *)malloc(sizeof(struct FL));
	tmp->frame_number = frame_number;
	tmp->next = NULL;
	if (FrameList_head == NULL) {
		FrameList_head = tmp;
	}
	else {
		tmp2 = FrameList_head;
		while (tmp2->next != NULL) {
			tmp2 = tmp2->next;
		}
		tmp2->next = tmp;
	}
}

/************************************************************************
	REMOVE_FROM_FREE_FRAME_LIST
		Get one free frame from a list initial in os_init.
		Success return frame number. Fail return -1.
		Input: 
		Output: frame number
*************************************************************************/
short remove_from_free_frame_list()
{
	FrameList tmp;
	if (FrameList_head == NULL) {
		return -1;
	}
	tmp = FrameList_head;
	FrameList_head = FrameList_head->next;
	tmp->next = NULL;
	return tmp->frame_number;
}

/************************************************************************
	MP_PRINT
		Call memory printer to print something.
		Input: void
		Output: void
************************************************************************/
void mp_print()
{
	short i;
	if (mp_full_print) {
		//if limited print is set-up and run out of the print limitation, do nothing
		if (mp_limited_print && mp_print_counter++ > 10) {
			return;
		}
		//else we normally setup the printer and print
		else {
			for (i=0; i<PHYS_MEM_PGS; i++) {
				if (mem_manager[i] != NULL) {
					MP_setup((INT32)(*mem_manager[i] & 0x0fff), (INT32)pid_storage[i], (INT32)(mem_manager[i] - address_storage[pid_storage[i]]), (INT32)((*mem_manager[i] & 0xf000) >> 13));
				}
			}
			MP_print_line();
		}
	}//if full print
}

/************************************************************************
	DISK_OPERATION
		If the operation is disk read, the information stored at <DISK_ID, logical_sector> is returned 
		in the variable pointed to by "data". If the operation is disk write, the inforamtion pointed 
		to by data is stored at <DISK_ID, logical_sector>. Note that this is a very low level system 
		call. In most Operating Systems, knowledge of the location of physical data is reserved for a 
		filesystem. The call as implemented here assumes that the user program understands the physical 
		disk. 
		Operation = 0, do disk read. Operation = 1, do disk write.
		DO_DISK_READ = 0. DO_DISK_WRITE = 1.
		Input: disk_id, logical_sector, data, operation
		Output: void
************************************************************************/
void disk_operation(int did, INT32 sector, char *data, INT32 operation)
{
	INT32 status;

	READ_MODIFY(DISK_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);

	//set up the disk
	MEM_WRITE(Z502DiskSetID, &did);
	MEM_READ(Z502DiskStatus, &status);
	if (status == DEVICE_FREE) {
		MEM_WRITE(Z502DiskSetSector, &sector);
		MEM_WRITE(Z502DiskSetBuffer, (INT32 *)data);
		status = operation;
		MEM_WRITE(Z502DiskSetAction, &status);
		status = 0;							//start the disk
		MEM_WRITE(Z502DiskStart, &status);

		current_pcb->disk_id = did;
		
		//move the pcb to suspend queue and waiting for disk operation complete
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

		is_suspend[current_pcb->process_id] = TRUE;
		add_to_suspend_queue_end(current_pcb);

		READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(DISK_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);
		
		dispatcher();
	}
	else if (status == DEVICE_IN_USE) {
		//store information that need to finish the operation
		current_pcb->disk_id = did;
		current_pcb->operation = operation;
		current_pcb->sector = sector;
		current_pcb->data = data;

		//move current pcb to suspend queue for a while
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

		is_suspend[current_pcb->process_id] = TRUE;
		add_to_suspend_queue_end(current_pcb);

		READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(DISK_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);
		
		dispatcher();
	}
	else {
		printf("Something wrong in disk operation %d!\n", operation);
		READ_MODIFY(DISK_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);
	}
}

/************************************************************************
	REMOVE_FROM_SUSPEND_QUEUE_BY_DID
		Remove first PCB with given disk id from suspend queue. If the suspend queue is empty already, 
		print a warning and return NULL.
		Input: did
		Output: pcb structure
*************************************************************************/
PCB *remove_from_suspend_queue_by_did(int did)
{
	TimerQueue tmp, tmp2;

	if (SuspendQueue_head == NULL) {
		printf("Suspend queue is empty!\n");
		return NULL;
	}
	tmp = SuspendQueue_head;
	if (tmp->data->disk_id == did) {
		SuspendQueue_head = tmp->next;
		return tmp->data;
	}
	else {
		while (tmp->next != NULL) {
			tmp2 = tmp;
			tmp = tmp->next;
			if (tmp->data->disk_id == did) {
				tmp2->next = tmp->next;
				return tmp->data;
			}
		}//while
	}
	return NULL;
}

/************************************************************************
    INTERRUPT_HANDLER
        When the Z502 gets a hardware interrupt, it transfers control to
        this routine in the OS.
************************************************************************/
void    interrupt_handler( void ) {
    INT32              device_id;
    INT32              status;
    INT32              Index = 0;
    //static BOOL        remove_this_in_your_code = TRUE;   /** TEMP **/
    //static INT32       how_many_interrupt_entries = 0;    /** TEMP **/
	PCB	*pcb;			//add
	INT32 sleep;		//add
	short did;			//add

    // Get cause of interrupt
    MEM_READ(Z502InterruptDevice, &device_id );

	while (device_id != -1) {
		// Set this device as target of our query
		MEM_WRITE(Z502InterruptDevice, &device_id );
		// Now read the status of this device
		MEM_READ(Z502InterruptStatus, &status );

		/** REMOVE THE NEXT SIX LINES **/
		//how_many_interrupt_entries++;                         /** TEMP **/
		//if ( remove_this_in_your_code && ( how_many_interrupt_entries < 20 ) )
		//    {
		//    printf( "Interrupt_handler: Found device ID %d with status %d\n",
		//                    device_id, status );
		//}

		if (device_id == 4) {
			/*============================   updated   =====================================*/
			READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
			pcb = remove_from_timer_queue();
	
			//i have to check if this pcb is suspended. if we remove the suspend process from timer q when
			//we did in suspend_process() instead of doing it here, a terrible problem happens when a interrupt
			//handler called but the target pcb is already in suspend q! because we cannot get its pid when it
			//goes to here!
			READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			if (!is_suspend[pcb->process_id]) {
				sp_print("Ready", pcb, NULL, NULL, FALSE, TRUE);
				add_to_ready_queue(pcb);
			}
			else {
				sp_print("Suspend", pcb, NULL, NULL, FALSE, TRUE);
				add_to_suspend_queue(pcb);
			}
			//check and move all items that have same or less wake up time to ready queue
			while (TimerQueue_head != NULL && TimerQueue_head->data->wake_up_time <= pcb->wake_up_time) {
				pcb = remove_from_timer_queue();
				//do what we do above
				if (!is_suspend[pcb->process_id]) {
					sp_print("Ready", pcb, NULL, NULL, FALSE, TRUE);
					add_to_ready_queue(pcb);
				}
				else {
					sp_print("Suspend", pcb, NULL, NULL, FALSE, TRUE);
					add_to_suspend_queue(pcb);
				}
			}
			READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);

			//reset the timer
			if (TimerQueue_head != NULL) {
				MEM_READ(Z502ClockStatus, &status);
				sleep = TimerQueue_head->data->wake_up_time - status;
				MEM_WRITE(Z502TimerStart, &sleep);
			}
			READ_MODIFY(TIMER_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
			/*=============================   updated end   =================================*/
		}
		/*================================    updated in phase2    ==========================*/
		else if (device_id >= 5 && device_id <= 16) {
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
			//disk id = process id - 4
			did = device_id - 4;
			pcb = remove_from_suspend_queue_by_did(did);
			//if that disk operation is not finished yet, continue that operation
			if (pcb != NULL && pcb->operation != DO_NOTHING) {
				READ_MODIFY(DISK_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);

				//do the disk operation again here
				MEM_WRITE(Z502DiskSetID, &pcb->disk_id);
				MEM_READ(Z502DiskStatus, &status);
				if (status == DEVICE_FREE) {
					MEM_WRITE(Z502DiskSetSector, &pcb->sector);
					MEM_WRITE(Z502DiskSetBuffer, (INT32 *)pcb->data);
					status = pcb->operation;	//DO_WRITE is 1; DO_READ is 0
					MEM_WRITE(Z502DiskSetAction, &status);
					status = 0;
					MEM_WRITE(Z502DiskStart, &status);
					//add to suspend queue
					pcb->operation = DO_NOTHING;
					add_to_suspend_queue_end(pcb);
				}
				else if (status == DEVICE_IN_USE) {
					printf("This should not happen!\n");
					add_to_suspend_queue(pcb);
				}

				READ_MODIFY(DISK_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);
			}
			//operation has done, move that pcb to ready queue
			else /*if (pcb != NULL && pcb->operation == DO_NOTHING)*/ {
				READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
				READ_MODIFY(IS_SUSPEND_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);

				add_to_ready_queue(pcb);
				is_suspend[pcb->process_id] = FALSE;

				READ_MODIFY(IS_SUSPEND_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &Is_SLock);
				READ_MODIFY(READY_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
			}
			READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		}//else if device id >= 5

		// Clear out this device - we're done with it
		MEM_WRITE(Z502InterruptClear, &Index );

		// Get cause of interrupt
		MEM_READ(Z502InterruptDevice, &device_id );
	}//while
}                                       /* End of interrupt_handler */

/************************************************************************
    FAULT_HANDLER
        The beginning of the OS502.  Used to receive hardware faults.
************************************************************************/

void    fault_handler( void )
    {
    INT32       device_id;
    INT32       status;
    INT32       Index = 0;

	//self added
	short frame_no, free_frame;
	BOOL debug_print = FALSE;
	//debug_print = TRUE;
	INT32 sector;
	
    // Get cause of interrupt
    MEM_READ(Z502InterruptDevice, &device_id );
    // Set this device as target of our query
    MEM_WRITE(Z502InterruptDevice, &device_id );
    // Now read the status of this device
    MEM_READ(Z502InterruptStatus, &status );

    printf( "Fault_handler: Found vector type %d with value %d\n",
                        device_id, status );

	/*===== used in test1k =====*/
	if (device_id == 4) {
		READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
		Z502Halt();
	}
	
	//if page table is empty, set up a page table
	if (Z502_PAGE_TBL_ADDR == NULL) {
		Z502_PAGE_TBL_LENGTH = 1024;
		Z502_PAGE_TBL_ADDR = (UINT16 *)calloc(sizeof(UINT16), Z502_PAGE_TBL_LENGTH);
		address_storage[current_pcb->process_id] = Z502_PAGE_TBL_ADDR;
	}

	//Invalid page table size so we halt the system
	if (status >= VIRTUAL_MEM_PGS || status < 0) {
		/*READ_MODIFY(TIMER_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &TimerQLock);
		READ_MODIFY(READY_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &ReadyQLock);
		READ_MODIFY(SUSPEND_QUEUE_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SuspendQLock);
		READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
		READ_MODIFY(DISK_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &DiskLock);*/
		printf("Invalid size of page table request.\n");
		Z502Halt();
	}

	//this address is valid
	if (Z502_PAGE_TBL_ADDR[status] >> 15 == 0) {
		//this address is not in disk
		if ((Z502_PAGE_TBL_ADDR[status] & PTBL_IN_DISK) >> 12 == 0) {
			//allocate a free frame for this address
			free_frame = remove_from_free_frame_list();
			//if there is free frame can be use
			if (free_frame != -1) {
				//set Physical Page Number to free frame number, and set this address invalid
				Z502_PAGE_TBL_ADDR[status] = free_frame | PTBL_VALID_BIT;
				mem_manager[mm_idx] = &Z502_PAGE_TBL_ADDR[status];
				pid_storage[mm_idx] = current_pcb->process_id;
				mm_idx++;
			}
			//there is no free frame can be allocated
			else {
				while (TRUE) {
					//modify the reference index and try to pick a victim
					ref_idx++;
					ref_idx %= PHYS_MEM_PGS;
					//if this address is referenced, set it to 0
					if ((*mem_manager[ref_idx] & PTBL_REFERENCED_BIT) >> 13 == 1) {
						(*mem_manager[ref_idx]) &= ~PTBL_REFERENCED_BIT;
					}
					//pick a victim
					else {
						//get the frame number from physical page number from that PAGE TBL ADDR
						frame_no = (short) *mem_manager[ref_idx] & 0x0fff;
						
						//swipe out, disk_id = pid+1, sector(status) = ADDR address-starting address, data = frame number*pagesize
						sector = mem_manager[ref_idx]-address_storage[pid_storage[ref_idx]];
						disk_operation(pid_storage[ref_idx]+1, sector, (char *)&MEMORY[frame_no * PGSIZE], DO_DISK_WRITE);
						if (debug_print) {
							printf("n_f_n_d==D==I==S==K==  status: %d, MEM: %d  ==W==R==I==T==E==\n", mem_manager[ref_idx]-address_storage[pid_storage[ref_idx]], frame_no * PGSIZE);
							//printf("*********  %s ***********\n", &MEMORY[frame_no*PGSIZE]);
						}
						
						//clear the physical page number and set this ADDR valid, referenced and in disk
						(*mem_manager[ref_idx]) |= PTBL_IN_DISK;
						(*mem_manager[ref_idx]) &= ~PTBL_VALID_BIT;

						//allocate frame number and set invalid
						Z502_PAGE_TBL_ADDR[status] = frame_no | PTBL_VALID_BIT;
						mem_manager[ref_idx] = &Z502_PAGE_TBL_ADDR[status];
						pid_storage[ref_idx] = current_pcb->process_id;
						break;
					}
				}//while
			}
		}
		//if this address in disk
		else {
			//find a free frame
			free_frame = remove_from_free_frame_list();
			if (free_frame != -1) {
				//read context from disk
				disk_operation(current_pcb->process_id+1, status, (char *)&MEMORY[free_frame * PGSIZE], DO_DISK_READ);
				if (debug_print) {
					printf("y_f_y_d==D==I==S==K==  status: %d, MEM: %d  ==R==E==A==D==\n", status, free_frame * PGSIZE);
					//printf("*********  %s ***********\n", MEMORY[free_frame*PGSIZE]);
				}

				//set Physical Page Number to free frame number, and set this address invalid
				Z502_PAGE_TBL_ADDR[status] = free_frame | PTBL_VALID_BIT;
				mem_manager[mm_idx] = &Z502_PAGE_TBL_ADDR[status];
				pid_storage[mm_idx] = current_pcb->process_id;
				mm_idx++;
			}
			//no free frame to allocate
			else {
				while (TRUE) {
					//modify reference index, and try to pick a victim
					ref_idx++;
					ref_idx %= PHYS_MEM_PGS;
					//if this address is referenced, set it to 0
					if ((*mem_manager[ref_idx]&PTBL_REFERENCED_BIT) >> 13 == 1) {
						(*mem_manager[ref_idx]) &= ~PTBL_REFERENCED_BIT;
					}
					//pick a victim
					else {
						//get the frame number from physical page number from that PAGE TBL ADDR
						frame_no = (short) *mem_manager[ref_idx] & 0x0fff;
						
						//swipe out, disk_id = pid+1, sector(status) = ADDR address-starting address, data = frame number*pagesize
						sector = mem_manager[ref_idx]-address_storage[pid_storage[ref_idx]];
						disk_operation(pid_storage[ref_idx]+1, sector, (char *)&MEMORY[frame_no * PGSIZE], DO_DISK_WRITE);
						if (debug_print) {
							printf("n_f_y_d==D==I==S==K==  status: %d, MEM: %d  ==W==R==I==T==E==\n", mem_manager[ref_idx]-address_storage[pid_storage[ref_idx]], frame_no * PGSIZE);
							//printf("*********  %s ***********\n", MEMORY[frame_no*PGSIZE]);
						}

						//clear the physical page number and set this ADDR valid, referenced and in disk
						(*mem_manager[ref_idx]) |= 0x1000;
						(*mem_manager[ref_idx]) &= ~PTBL_VALID_BIT;
						
						//swipe in
						disk_operation(current_pcb->process_id+1, status, (char *)&MEMORY[frame_no * PGSIZE], DO_DISK_READ);
						if (debug_print) {
							printf("n_f_y_d==D==I==S==K==  status: %d, MEM: %d  ==R==E==A==D==\n", status, frame_no * PGSIZE);
							//printf("*********  %s ***********\n", MEMORY[frame_no*PGSIZE]);
						}

						//allocate frame number and set invalid
						Z502_PAGE_TBL_ADDR[status] = frame_no | PTBL_VALID_BIT;
						mem_manager[ref_idx] = &Z502_PAGE_TBL_ADDR[status];
						pid_storage[ref_idx] = current_pcb->process_id;
						break;
					}
				}//while
			}
		}
	}

	mp_print();

	// Clear out this device - we're done with it
	MEM_WRITE(Z502InterruptClear, &Index );
}                                       /* End of fault_handler */

/************************************************************************
    SVC
        The beginning of the OS502.  Used to receive software interrupts.
        All system calls come to this point in the code and are to be
        handled by the student written code here.
        The variable do_print is designed to print out the data for the
        incoming calls, but does so only for the first ten calls.  This
        allows the user to see what's happening, but doesn't overwhelm
        with the amount of data.
************************************************************************/

void    svc( SYSTEM_CALL_DATA *SystemCallData ) {
    short               call_type;
    static short        do_print = 10;
    short               i;
	INT32				Time;

    call_type = (short)SystemCallData->SystemCallNumber;
    if ( do_print > 0 ) {
        printf( "SVC handler: %s\n", call_names[call_type]);
        for (i = 0; i < SystemCallData->NumberOfArguments - 1; i++ ){
        	 //Value = (long)*SystemCallData->Argument[i];
             printf( "Arg %d: Contents = (Decimal) %8ld,  (Hex) %8lX\n", i,
             (unsigned long )SystemCallData->Argument[i],
             (unsigned long )SystemCallData->Argument[i]);
        }
		do_print--;
    }

	switch (call_type) {
		case SYSNUM_GET_TIME_OF_DAY:
			MEM_READ(Z502ClockStatus, &Time);
			*(INT32 *)SystemCallData->Argument[0] = Time;
			READ_MODIFY(SP_PRINTER_MEMORY, DO_LOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			READ_MODIFY(SP_PRINTER_MEMORY, DO_UNLOCK, SUSPEND_UNTIL_LOCKED, &SPLock);
			break;
		case SYSNUM_TERMINATE_PROCESS:
			terminate_process((int)SystemCallData->Argument[0], (INT32 *)SystemCallData->Argument[1]);
			break;
		case SYSNUM_SLEEP:
			start_timer((INT32)SystemCallData->Argument[0]);
			break;
		case SYSNUM_CREATE_PROCESS:
			create_process(SystemCallData->Argument[0], SystemCallData->Argument[1], (int)SystemCallData->Argument[2], (INT32 *)SystemCallData->Argument[3], (INT32 *)SystemCallData->Argument[4]);
			break;
		case SYSNUM_GET_PROCESS_ID:
			get_process_id(SystemCallData->Argument[0], (INT32 *)SystemCallData->Argument[1], (INT32 *)SystemCallData->Argument[2]);
			break;
		case SYSNUM_SUSPEND_PROCESS:
			suspend_process((int)SystemCallData->Argument[0], (INT32 *)SystemCallData->Argument[1]);
			break;
		case SYSNUM_RESUME_PROCESS:
			resume_process((int)SystemCallData->Argument[0], (INT32 *)SystemCallData->Argument[1]);
			break;
		case SYSNUM_CHANGE_PRIORITY:
			change_priority((int)SystemCallData->Argument[0], (int)SystemCallData->Argument[1], (INT32 *)SystemCallData->Argument[2]);
			break;
		case SYSNUM_SEND_MESSAGE:
			send_message((int)SystemCallData->Argument[0], SystemCallData->Argument[1], (INT32)SystemCallData->Argument[2], (INT32 *)SystemCallData->Argument[3]);
			break;
		case SYSNUM_RECEIVE_MESSAGE:
			receive_message((int)SystemCallData->Argument[0], SystemCallData->Argument[1], (INT32)SystemCallData->Argument[2], (INT32 *)SystemCallData->Argument[3], (INT32 *)SystemCallData->Argument[4], (INT32 *)SystemCallData->Argument[5]);
			break;
		case SYSNUM_DISK_READ:
			disk_operation((int)SystemCallData->Argument[0], (INT32)SystemCallData->Argument[1], (char *)SystemCallData->Argument[2], DO_DISK_READ);
			break;
		case SYSNUM_DISK_WRITE:
			disk_operation((int)SystemCallData->Argument[0], (INT32)SystemCallData->Argument[1], (char *)SystemCallData->Argument[2], DO_DISK_WRITE);
			break;
		default:
			printf("ERROR! call_type not recognized!\n");
			printf("Call_type is - %i\n", call_type);
	}

}                                               // End of svc

/************************************************************************
    osInit
        This is the first routine called after the simulation begins.  This
        is equivalent to boot code.  All the initial OS components can be
        defined and initialized here.
************************************************************************/

void    osInit( int argc, char *argv[]  ) {
    void                *next_context;
    INT32               i;
	
    /* Demonstrates how calling arguments are passed thru to here       */

    printf( "Program called with %d arguments:", argc );
    for ( i = 0; i < argc; i++ )
        printf( " %s", argv[i] );
    printf( "\n" );
    printf( "Calling with argument 'sample' executes the sample program.\n" );

    /*          Setup so handlers will come to code in base.c           */

    TO_VECTOR[TO_VECTOR_INT_HANDLER_ADDR]   = (void *)interrupt_handler;
    TO_VECTOR[TO_VECTOR_FAULT_HANDLER_ADDR] = (void *)fault_handler;
    TO_VECTOR[TO_VECTOR_TRAP_HANDLER_ADDR]  = (void *)svc;

    /*  Determine if the switch was set, and if so go to demo routine.  */

    if (( argc > 1 ) && ( strcmp( argv[1], "sample" ) == 0 ) ) {
        Z502MakeContext( &next_context, (void *)sample_code, KERNEL_MODE );
        Z502SwitchContext( SWITCH_CONTEXT_KILL_MODE, &next_context );
    }                   /* This routine should never return!!           */

    /*  This should be done by a "make_process" routine, so that
        test0 runs on a process recognized by the operating system.    */
    //Z502MakeContext( &next_context, (void *)test1a, USER_MODE );
    //Z502SwitchContext( SWITCH_CONTEXT_KILL_MODE, &next_context );

	//phase1 initial
	TimerQueue_head = NULL;
	ReadyQueue_head = NULL;
	SuspendQueue_head = NULL;
	sp_full_print = FALSE;
	mp_full_print = FALSE;
	pno = 0;
	msg_counter = 0;
	//phase2 initial
	mm_idx = 0;
	ref_idx = -1;
	sp_limited_print = FALSE;
	mp_limited_print = FALSE;
	sp_print_counter = mp_print_counter = 0;

	for (pid=0; pid<100; pid++) {
		valid_pid[pid] = TRUE;
		is_suspend[pid] = FALSE;
	}

	for (pid = 0; pid < PHYS_MEM_PGS; pid++) {
		add_to_free_frame_list((short) pid);
		mem_manager[pid] = NULL;
	}


	pid = 0;

	if ((argc > 1) && (strcmp(argv[1], "test0") == 0))
		current_pcb = make_process(0, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1a") == 0))
		current_pcb = make_process(1, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1b") == 0))
		current_pcb = make_process(2, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1c") == 0))
		current_pcb = make_process(3, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1d") == 0))
		current_pcb = make_process(4, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1e") == 0))
		current_pcb = make_process(5, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1f") == 0))
		current_pcb = make_process(6, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1g") == 0))
		current_pcb = make_process(7, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1h") == 0))
		current_pcb = make_process(8, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1i") == 0))
		current_pcb = make_process(9, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1j") == 0))
		current_pcb = make_process(10, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1k") == 0))
		current_pcb = make_process(11, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1l") == 0))
		current_pcb = make_process(12, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test1m") == 0))
		current_pcb = make_process(13, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2a") == 0))
		current_pcb = make_process(20, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2b") == 0))
		current_pcb = make_process(21, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2c") == 0))
		current_pcb = make_process(22, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2d") == 0))
		current_pcb = make_process(23, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2e") == 0))
		current_pcb = make_process(24, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2f") == 0))
		current_pcb = make_process(25, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2g") == 0))
		current_pcb = make_process(26, argv[1]);
	else if ((argc > 1) && (strcmp(argv[1], "test2h") == 0))
		current_pcb = make_process(27, argv[1]);
	else
		current_pcb = make_process(-1, "test0");

	sp_print("OS_Init", current_pcb, current_pcb, NULL, FALSE, FALSE);

	Z502SwitchContext(SWITCH_CONTEXT_KILL_MODE, &(current_pcb->context));
}                                               // End of osInit
