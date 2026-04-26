#ifndef UART_H
#define UART_H

struct GazeCommand {
    float xPos;
    float yPos;
    bool grab;
};

extern GazeCommand gazeCommand;

extern float min_working_x;
extern float max_working_x;
extern float min_working_y;
extern float max_working_y;

void initializeUART();
void parseUART();

#endif // UART_H