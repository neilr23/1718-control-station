#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"
#include "windows.h"

#define FILE_MENU_REFRESH 1
#define FILE_MENU_OPEN 2
#define MENU_RESET 4
#define MENU_HELP 5
#define FILE_MENU_EXIT 6

using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char port_name[] = "\\\\.\\COM3";
HWND TextBox;
char textSaved[2];
SerialPort *robot = new SerialPort(port_name);

//String for incoming data
char incomingData[MAX_DATA_LENGTH];

//Window Definitions
LRESULT CALLBACK WindowProcedure (HWND, UINT, WPARAM, LPARAM);

//Exception Handling Function
void *__gxx_personality_v0;
void *_ZdlPvj;

//Window Class Name
char szClassName[ ] = "WindowsApp";

//Menus
void AddMenus(HWND);
bool ComPrompt = false;

//Communications Functions
void ComLoop();
void Connect();
   
HMENU hMenu;

int WINAPI WinMain (HINSTANCE hInst, HINSTANCE hPrevInst, LPSTR args, int cmdshow)
{
   WNDCLASSW wc = {0};
   
   wc.hbrBackground = (HBRUSH)COLOR_WINDOW;
   wc.hCursor = LoadCursor(NULL, IDC_ARROW);
   wc.hInstance = hInst;
   wc.lpszClassName = L"myWindowClass";
   wc.lpfnWndProc = WindowProcedure;
   
   if(!RegisterClassW(&wc))
      return -1;
   
   CreateWindowW(L"myWindowClass", L"TJHSST ROV GUI 2018", WS_OVERLAPPEDWINDOW | WS_VISIBLE
      , 100, 100, 500, 500, NULL, NULL, NULL, NULL);
   
   MSG msg = {0};
   
   while( GetMessage(&msg, NULL, 0, 0) )
   {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
   }
   
   return (int)msg.wParam;
}

//Message Handler
LRESULT CALLBACK WindowProcedure(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp)
{
   
   switch (msg)
   {
      case WM_COMMAND:
         switch(wp)
         {
            case FILE_MENU_REFRESH:
               MessageBeep(MB_OK);
               ComLoop();
               break;
            case FILE_MENU_OPEN:
            
               break;
            case 3:
               if(!ComPrompt)
               {
                  ShowWindow (GetConsoleWindow(), SW_SHOW);
                  ComPrompt = true;   
               }
               else
               {
                  ShowWindow (GetConsoleWindow(), SW_HIDE);
                  ComPrompt = false;  
               }
               break;
            case MENU_RESET:
               system("CLS");
               break;
            case MENU_HELP:
               MessageBox(hWnd, 
               "TJHSST ROV GUI 2018\n\nFile: Contains Refresh, Open, and Exit\n\tRefresh: Gets latest data from current port\n\tOpen: Opens new specified port\n\tExit: Closes program\nCommand: Opens or closes command prompt\nReset: Clears text in command prompt",
               "Help", MB_OK);
               break;
            case FILE_MENU_EXIT:
               PostQuitMessage(0);
               break;
         }
         break;
      case WM_CREATE:
         ShowWindow (GetConsoleWindow(), SW_HIDE);
         Connect();
         AddMenus(hWnd);
         TextBox = CreateWindow("EDIT", "Input COM Port", WS_BORDER | WS_CHILD | WS_VISIBLE, 10, 10, 400, 20, hWnd, NULL, NULL, NULL);
         break;
      case WM_DESTROY:
         PostQuitMessage(0);
         break;
      default:
         return DefWindowProcW(hWnd,msg,wp,lp);
   }
}

void AddMenus(HWND hWnd)
{
   hMenu = CreateMenu();
   HMENU hFileMenu = CreateMenu();
   
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_REFRESH, "Refresh");
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_OPEN, "Open");
   AppendMenu(hFileMenu, MF_SEPARATOR, 0, NULL);
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_EXIT, "Exit");
   
   AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, "File");
   AppendMenu(hMenu, MF_STRING, 3, "Command");
   AppendMenu(hMenu, MF_STRING, MENU_RESET, "Reset");
   AppendMenu(hMenu, MF_STRING, MENU_HELP, "Help");
   
   SetMenu(hWnd, hMenu); 
}

void Connect()
{
   robot = new SerialPort(port_name);
   if(robot->isConnected()) 
      cout << "Connection Established" << endl;
   else 
      cout << "ERROR, check port name" << endl;
}

void ComLoop()
{
   if(robot->isConnected())
   {
      int read_result = robot->readSerialPort(incomingData, MAX_DATA_LENGTH);
      //prints out data
      cout << incomingData << endl;
   }
   else
      cout << "Not Connected" << endl;
   return; 
}