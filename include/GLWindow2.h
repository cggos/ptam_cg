// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __GL_WINDOW_2_H
#define __GL_WINDOW_2_H

#include <vector>
#include <map>
#include <cvd/glwindow.h>
#include <TooN/TooN.h>
#include <gvars3/gvars3.h>

class GLWindow2;

// A simple gvars-driven menu system for GLWindow2
// N.b. each GLWindowMenu class internally contains sub-menus
class GLWindowMenu
{
public:
    GLWindowMenu(std::string sName, std::string sTitle);
    ~GLWindowMenu();
    void Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw);
    void FillBox(int l, int r, int t, int b);
    void LineBox(int l, int r, int t, int b);

    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

    bool HandleClick(int button, int state, int x, int y);

private:
    enum MenuItemType { Button, Toggle, Monitor, Slider };

    struct MenuItem
    {
        MenuItemType type;
        std::string sName;
        std::string sParam;
        std::string sNextMenu;
        GVars3::gvar2_int gvnIntValue;  // Not used by all, but used by some
        int min;
        int max;
    };

    struct SubMenu
    {
        std::vector<MenuItem> mvItems;
    };

    std::map<std::string, SubMenu> mmSubMenus;
    std::string msCurrentSubMenu;
    std::string msName;
    std::string msTitle;


    int mnWidth;
    int mnMenuTop;
    int mnMenuHeight;
    int mnTextOffset;

    GVars3::gvar2_int mgvnEnabled;
    GVars3::gvar2_int mgvnMenuItemWidth;
    GVars3::gvar2_int mgvnMenuTextOffset;

    int mnLeftMostCoord;
};

//  A class which wraps a CVD::GLWindow and provides some basic
//  user-interface funtionality: A gvars-driven clickable menu, and a
//  caption line for text display. Also provides some handy GL helpers
//  and a wrapper for CVD's text display routines.
class GLWindow2 : public CVD::GLWindow, public CVD::GLWindow::EventHandler
{
public:
    GLWindow2(CVD::ImageRef irSize, std::string sTitle);

    // The preferred event handler..
    void HandlePendingEvents();

    // Menu interface:
    void AddMenu(std::string sName, std::string sTitle);
    void DrawMenus();

    // Some OpenGL helpers:
    void SetupViewport();
    void SetupVideoOrtho();
    void SetupUnitOrtho();
    void SetupWindowOrtho();
    void SetupVideoRasterPosAndZoom();

    // Text display functions:
    void PrintString(CVD::ImageRef irPos, std::string s);
    void DrawCaption(std::string s);

    // Map viewer mouse interface:
    std::pair<TooN::Vector<6>, TooN::Vector<6> > GetMousePoseUpdate();

protected:
    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

    // User interface menus:
    std::vector<GLWindowMenu*> mvpGLWindowMenus;

    CVD::ImageRef mirVideoSize;   // The size of the source video material.


    // Event handling routines:
    virtual void on_key_down(GLWindow&, int key);
    virtual void on_mouse_move(GLWindow& win, CVD::ImageRef where, int state);
    virtual void on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button);
    virtual void on_event(GLWindow& win, int event);
    CVD::ImageRef mirLastMousePos;

    // Storage for map viewer updates:
    TooN::Vector<6> mvMCPoseUpdate;
    TooN::Vector<6> mvLeftPoseUpdate;
};

#endif
