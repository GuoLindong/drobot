#ifndef SERIAL_H_
#define SERIAL_H_

int OpenSerial(void **handle, const char *port_name);

int SetupSerial(void *handle);

int WriteData(void *handle, const char *buffer, int length);

int ReadData(void *handle, char *buffer, int length);

int CloseSerial(void *handle);

#endif /* SERIAL_H_ */