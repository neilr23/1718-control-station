#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"
#include "windows.h"

#define FILE_MENU_NEW 1
#define FILE_MENU_OPEN 2
#define FILE_MENU_EXIT 6

using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char port_name[] = "\\\\.\\COM20";
HWND TextBox;
char textSaved[2];
SerialPort arduino(port_name);

//String for incoming data
char incomingData[MAX_DATA_LENGTH];

//Window Definitions
LRESULT CALLBACK WindowProcedure (HWND, UINT, WPARAM, LPARAM);

//Exception Handling Function
void *__gxx_personality_v0;

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
   
   CreateWindowW(L"myWindowClass", L"TJHSST ROV 2018", WS_OVERLAPPEDWINDOW | WS_VISIBLE
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
            case FILE_MENU_NEW:
               MessageBeep(MB_OK);
               ComLoop();
               break;
            case FILE_MENU_OPEN:
               int gwtstat;
               char *t = &textSaved[0];
               gwtstat = GetWindowText(TextBox, t, 2);
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
            case 4:
               break;
            case 5:
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
   
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_NEW, "New");
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_OPEN, "Open");
   AppendMenu(hFileMenu, MF_SEPARATOR, 0, NULL);
   AppendMenu(hFileMenu, MF_STRING, FILE_MENU_EXIT, "Exit");
   
   AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, "File");
   AppendMenu(hMenu, MF_STRING, 3, "Command");
   AppendMenu(hMenu, MF_STRING, 4, "Reset");
   AppendMenu(hMenu, MF_STRING, 5, "Help");
   
   SetMenu(hWnd, hMenu); 
}

void Connect()
{
   //arduino = new SerialPort arduino(port_name);
   if(arduino.isConnected()) 
      cout << "Connection Established" << endl;
   else 
      cout << "ERROR, check port name" << endl;
}

void ComLoop()
{
   if(arduino.isConnected())
   {
      int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
      //prints out data
      cout << incomingData << endl;
   }
   else
      cout << "Not Connected" << endl;
      return; 
}