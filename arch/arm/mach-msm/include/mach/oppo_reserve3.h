/* 
 *
 * yixue.ge add for oppo custom partion
 *
 * do`nt modify this file,if you want add some diffrent device please add at diff_project.h
 */
#ifndef _OPPO_RESERVE3_H_
#define _OPPO_RESERVE3_H_

#define OPFS_OPPOMAGIC1 0x6f706673
#define OPFS_OPPOMAGIC2 0x67657978
#define RESERVE3_LABEL "reserve3"
#define RESERVE3_MAX_SIZE 6*1024*1024


typedef struct
{
  unsigned int                  oppomagic1;
  unsigned int                  oppomagic2;
  unsigned int				   	gama_flag;//	
} reserve3_data;

void init_reserve3(void);
unsigned int get_gamaflag(void);

#endif

