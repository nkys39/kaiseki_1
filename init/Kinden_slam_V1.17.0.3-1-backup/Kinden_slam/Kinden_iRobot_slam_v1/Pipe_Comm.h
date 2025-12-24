#if !defined(PIPE_COMM_HEADER)
#define PIPE_COMM_HEADER

int PipeCreate(void);
void PipeCommReceive(void);
void PipeCommSend(char *szbuff);
void MapFileNameReceive(void);
void StatusSet(char);
#endif