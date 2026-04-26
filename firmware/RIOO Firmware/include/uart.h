#ifndef UART_H
#define UART_H

struct GazeCommand {
    float xPos;
    float yPos;
    bool grab;
};

extern GazeCommand gazeCommand;

void initializeUART();
void parseUART();

#endif // UART_H