//
// Created by nickolay on 12.04.2020.
//

#include "MAVConnector.h"
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 60239


MAVConnector::MAVConnector()
{

}

bool MAVConnector::ConnectToServer()
{
    bool result = true;
    int valread = 0;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        result = false;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        result = false;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        result = false;
    }
    return result;
}

void MAVConnector::SendData(char *data)
{
    send(sock , data , strlen(data) , 0 );
}

void MAVConnector::test()
{
    /*std::cout<<"START TEST"<<std::endl;

    setenv("PYTHONPATH", "..", 1);

    Py_Initialize();

    PyObject* module = PyImport_ImportModule("MAVConnecor");
    assert(module != NULL);

    PyObject* klass = PyObject_GetAttrString(module, "testMav");
    assert(klass != NULL);

    PyObject* instance = PyInstance_New(klass, NULL, NULL);
    assert(instance != NULL);

    PyObject* result = PyObject_CallMethod(instance, "test", "()");
    assert(result != NULL);

    std::cout<<result<<std::endl;
    //printf("1 + 2 = %ld\n", PyInt_AsLong(result));

    Py_Finalize();*/
}
