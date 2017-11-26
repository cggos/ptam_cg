#include "OpenGL.h"
#include "GLWindow2.h"

#include <stdlib.h>
#include <sstream>
#include <gvars3/GStringUtil.h>
#include <gvars3/instances.h>
#include <TooN/helpers.h>

using namespace std;
using namespace CVD;
using namespace GVars3;
using namespace TooN;

GLWindowMenu::GLWindowMenu(string sName, string sTitle)
{
    msName = sName;
    msTitle = sTitle;

    GUI.RegisterCommand(msName+".AddMenuButton", GUICommandCallBack, this);
    GUI.RegisterCommand(msName+".AddMenuToggle", GUICommandCallBack, this);
    GUI.RegisterCommand(msName+".AddMenuSlider", GUICommandCallBack, this);
    GUI.RegisterCommand(msName+".AddMenuMonitor", GUICommandCallBack, this);
    GUI.RegisterCommand(msName+".ShowMenu", GUICommandCallBack, this);
    GV2.Register(mgvnMenuItemWidth, msName+".MenuItemWidth", 90, HIDDEN | SILENT);
    GV2.Register(mgvnMenuTextOffset, msName+".MenuTextOffset", 20, HIDDEN | SILENT);
    GV2.Register(mgvnEnabled, msName+".Enabled", 1, HIDDEN | SILENT);

    mmSubMenus.clear();
    msCurrentSubMenu="";
}

GLWindowMenu::~GLWindowMenu()
{
    GUI.UnRegisterCommand(msName+".AddMenuButton");
    GUI.UnRegisterCommand(msName+".AddMenuToggle");
    GUI.UnRegisterCommand(msName+".AddMenuSlider");
    GUI.UnRegisterCommand(msName+".AddMenuMonitor");
    GUI.UnRegisterCommand(msName+".ShowMenu");
};


void GLWindowMenu::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
    ((GLWindowMenu*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindowMenu::GUICommandHandler(string sCommand, string sParams)
{
    vector<string> vs = ChopAndUnquoteString(sParams);

    if(sCommand==msName+".AddMenuButton")
    {
        if(vs.size()<3)
        {
            cout<< "? GLWindowMenu.AddMenuButton: Need 3/4 params: Target Menu, Name, Command , NextMenu=\"\"" << endl;
            return;
        };
        MenuItem m;
        m.type = Button;
        m.sName = vs[1];
        m.sParam = UncommentString(vs[2]);
        m.sNextMenu = (vs.size()>3)?(vs[3]):("");
        mmSubMenus[vs[0]].mvItems.push_back(m);
        return;
    }

    if(sCommand==msName+".AddMenuToggle")
    {
        if(vs.size()<3)
        {
            cout<< "? GLWindowMenu.AddMenuToggle: Need 3/4 params: Target Menu, Name, gvar_int name , NextMenu=\"\"" << endl;
            return;
        };
        MenuItem m;
        m.type = Toggle;
        m.sName = vs[1];
        GV2.Register(m.gvnIntValue, vs[2]);
        m.sNextMenu = (vs.size()>3)?(vs[3]):("");
        mmSubMenus[vs[0]].mvItems.push_back(m);
        return;
    }

    if(sCommand==msName+".AddMenuMonitor")
    {
        if(vs.size()<3)
        {
            cout<< "? GLWindowMenu.AddMenuMonitor: Need 3/4 params: Target Menu, Name, gvar name , NextMenu=\"\"" << endl;
            return;
        };
        MenuItem m;
        m.type = Monitor;
        m.sName = vs[1];
        m.sParam = vs[2];
        m.sNextMenu = (vs.size()>3)?(vs[3]):("");
        mmSubMenus[vs[0]].mvItems.push_back(m);
        return;
    }

    if(sCommand==msName+".AddMenuSlider")
    {
        if(vs.size()<5)
        {
            cout<< "? GLWindowMenu.AddMenuSlider: Need 5/6 params: Target Menu, Name, gvar_int name, min, max, NextMenu=\"\"" << endl;
            return;
        };
        MenuItem m;
        m.type = Slider;
        m.sName = vs[1];
        GV2.Register(m.gvnIntValue, vs[2]);
        int *a;
        a = ParseAndAllocate<int>(vs[3]);
        if(a)
        {
            m.min = *a;
            delete a;
        }
        a = ParseAndAllocate<int>(vs[4]);
        if(a)
        {
            m.max = *a;
            delete a;
        }
        m.sNextMenu = (vs.size()>5)?(vs[5]):("");
        mmSubMenus[vs[0]].mvItems.push_back(m);
        return;
    }

    if(sCommand==msName+".ShowMenu")
    {
        if(vs.size()==0)
            msCurrentSubMenu = "";
        else
            msCurrentSubMenu = vs[0];
    };

};

void GLWindowMenu::LineBox(int l, int r, int t, int b)
{
    glBegin(GL_LINE_STRIP);
    glVertex2i(l,t);
    glVertex2i(l,b);
    glVertex2i(r,b);
    glVertex2i(r,t);
    glVertex2i(l,t);
    glEnd();
}

void GLWindowMenu::FillBox(int l, int r, int t, int b)
{
    glBegin(GL_QUADS);
    glVertex2i(l,t);
    glVertex2i(l,b);
    glVertex2i(r,b);
    glVertex2i(r,t);
    glEnd();
}

void GLWindowMenu::Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw)
{
    if(!*mgvnEnabled)
        return;

    mnWidth = nWidth;
    mnMenuTop = nTop;
    mnMenuHeight = nHeight;

    double dAlpha = 0.8;
    if(msCurrentSubMenu=="")  // No Menu selected  - draw little arrow.
    {
        glColor4d(0,0.5,0,0.5);
        FillBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
        glColor4d(0,1,0,0.5);
        LineBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
        mnLeftMostCoord = mnWidth - 30;
        return;
    };

    SubMenu &m = mmSubMenus[msCurrentSubMenu];

    mnLeftMostCoord = mnWidth - (1 + m.mvItems.size()) * *mgvnMenuItemWidth;
    int nBase = mnLeftMostCoord;
    for(vector<MenuItem>::reverse_iterator i = m.mvItems.rbegin(); i!= m.mvItems.rend(); i++)
    {
        switch(i->type)
        {
        case Button:
            glColor4d(0,0.5,0,dAlpha);
            FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glColor4d(0,1,0,dAlpha);
            LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                            i->sName);
            break;

        case Toggle:
            if(*(i->gvnIntValue))
                glColor4d(0,0.5,0.5,dAlpha);
            else
                glColor4d(0.5,0,0,dAlpha);
            FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            if(*(i->gvnIntValue))
                glColor4d(0,1,0.5,dAlpha);
            else
                glColor4d(1,0,0,dAlpha);
            LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                            i->sName + " " + ((*(i->gvnIntValue))?("On"):("Off")));
            break;

        case Monitor:
            glColor4d(0,0.5,0.5,dAlpha);
            FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glColor4d(0,1,1,dAlpha);
            LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                            i->sName + " " + GV2.StringValue(i->sParam, true));
            break;

        case Slider:
        {
            glColor4d(0.0,0.0,0.5,dAlpha);
            FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            glColor4d(0.5,0.0,0.5,dAlpha);
            double dFrac = (double) (*(i->gvnIntValue) - i->min) / (i->max - i->min);
            if(dFrac<0.0)
                dFrac = 0.0;
            if(dFrac>1.0)
                dFrac = 1.0;
            FillBox(nBase, (int) (nBase + dFrac * (*mgvnMenuItemWidth +1)), mnMenuTop, mnMenuTop + mnMenuHeight);
            glColor4d(0,1,1,dAlpha);
            LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
            ostringstream ost;
            ost << i->sName << " " << *(i->gvnIntValue);
            glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                            ost.str());
        }
            break;
        }
        nBase += *mgvnMenuItemWidth;
    };
    glColor4d(0.5, 0.5, 0,dAlpha);
    FillBox(mnWidth - *mgvnMenuItemWidth, mnWidth-1, mnMenuTop, mnMenuTop + mnMenuHeight);
    glColor4d(1,1,0,dAlpha);
    LineBox(mnWidth - *mgvnMenuItemWidth, mnWidth-1, mnMenuTop, mnMenuTop + mnMenuHeight);
    ImageRef ir( mnWidth - *mgvnMenuItemWidth + 5, mnMenuTop + *mgvnMenuTextOffset);
    if(msCurrentSubMenu == "Root")
        glw.PrintString(ir, msTitle+":");
    else
        glw.PrintString(ir, msCurrentSubMenu+":");
};


bool GLWindowMenu::HandleClick(int nMouseButton, int state, int x, int y)
{
    if(!*mgvnEnabled)
        return false;

    if((y<mnMenuTop)||(y>mnMenuTop + mnMenuHeight))
        return false;
    if(x<mnLeftMostCoord)
        return false;

    // if no menu displayed, then must display root menu!
    if(msCurrentSubMenu == "")
    {
        msCurrentSubMenu = "Root";
        return true;
    };

    // Figure out which button was pressed:
    int nButtonNumber = (mnWidth - x) / *mgvnMenuItemWidth;
    if(nButtonNumber > (int)(mmSubMenus[msCurrentSubMenu].mvItems.size()))
        nButtonNumber = 0;

    if(nButtonNumber==0) // Clicked on menu name .. . go to root.
    {
        if(msCurrentSubMenu =="Root")
            msCurrentSubMenu = "";
        else
            msCurrentSubMenu = "Root";
        return true;
    };

    MenuItem SelectedItem  = mmSubMenus[msCurrentSubMenu].mvItems[nButtonNumber-1];
    msCurrentSubMenu=SelectedItem.sNextMenu;
    switch(SelectedItem.type)
    {
    case Button:
        GUI.ParseLine(SelectedItem.sParam);
        break;
    case Toggle:
        *(SelectedItem.gvnIntValue)^=1;
        break;
    case Slider:
    {
        if(nMouseButton == GLWindow::BUTTON_WHEEL_UP)
        {
            *(SelectedItem.gvnIntValue)+=1;
            if(*(SelectedItem.gvnIntValue) > SelectedItem.max)
                *(SelectedItem.gvnIntValue) = SelectedItem.max;
        }
        else if(nMouseButton == GLWindow::BUTTON_WHEEL_DOWN)
        {
            *(SelectedItem.gvnIntValue)-=1;
            if(*(SelectedItem.gvnIntValue) < SelectedItem.min)
                *(SelectedItem.gvnIntValue) = SelectedItem.min;
        }
        else
        {
            int nPos = *mgvnMenuItemWidth - ((mnWidth - x) % *mgvnMenuItemWidth);
            double dFrac = (double) nPos / *mgvnMenuItemWidth;
            *(SelectedItem.gvnIntValue) = (int)(dFrac * (1.0 + SelectedItem.max - SelectedItem.min)) + SelectedItem.min;
        };
    }
        break;
    case Monitor:
        break;
    };
    return true;

}



GLWindow2::GLWindow2(ImageRef irSize, string sTitle)
    : GLWindow(irSize, sTitle)
{

#ifdef WIN32
    // On windows, have to initialise GLEW at the start to enable access
    // to GL extensions
    static bool bGLEWIsInit = false;
    if(!bGLEWIsInit)
    {
        GLenum err = glewInit();
        if (GLEW_OK != err)
        {
            fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(err));
            exit(0);
        }
        bGLEWIsInit = true;
    }
#endif

    mirVideoSize = irSize;
    GUI.RegisterCommand("GLWindow.AddMenu", GUICommandCallBack, this);
    glSetFont("sans");
    mvMCPoseUpdate=Zeros;
    mvLeftPoseUpdate=Zeros;
};


void GLWindow2::AddMenu(string sName, string sTitle)
{
    GLWindowMenu* pMenu = new GLWindowMenu(sName, sTitle);
    mvpGLWindowMenus.push_back(pMenu);
}

void GLWindow2::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
    ((GLWindow2*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindow2::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
    vector<string> vs=ChopAndUnquoteString(sParams);
    if(sCommand=="GLWindow.AddMenu")
    {
        switch(vs.size())
        {
        case 1:
            AddMenu(vs[0], "Root");
            return;
        case 2:
            AddMenu(vs[0], vs[1]);
            return;
        default:
            cout << "? AddMenu: need one or two params (internal menu name, [caption])." << endl;
            return;
        };
    };

    // Should have returned to caller by now - if got here, a command which
    // was not handled was registered....
    cout << "! GLWindow::GUICommandHandler: unhandled command "<< sCommand << endl;
    exit(1);
}; 

void GLWindow2::DrawMenus()
{
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_POLYGON_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMask(1,1,1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    SetupWindowOrtho();
    glLineWidth(1);

    int nTop = 30;
    int nHeight = 30;
    for(vector<GLWindowMenu*>::iterator i = mvpGLWindowMenus.begin();
        i!= mvpGLWindowMenus.end();
        i++)
    {
        (*i)->Render(nTop, nHeight, size()[0], *this);
        nTop+=nHeight+1;
    }

}

void GLWindow2::SetupUnitOrtho()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,1,1,0,0,1);
}

void GLWindow2::SetupWindowOrtho()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(size());
}

void GLWindow2::SetupVideoOrtho()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5,(double)mirVideoSize.x - 0.5, (double) mirVideoSize.y - 0.5, -0.5, -1.0, 1.0);
}

void GLWindow2::SetupVideoRasterPosAndZoom()
{ 
    glRasterPos2d(-0.5,-0.5);
    double adZoom[2];
    adZoom[0] = (double) size()[0] / (double) mirVideoSize[0];
    adZoom[1] = (double) size()[1] / (double) mirVideoSize[1];
    glPixelZoom(adZoom[0], -adZoom[1]);
}

void GLWindow2::SetupViewport()
{
    glViewport(0, 0, size()[0], size()[1]);
}

void GLWindow2::PrintString(CVD::ImageRef irPos, std::string s)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glTranslatef(irPos.x, irPos.y, 0.0);
    glScalef(8,-8,1);
    glDrawText(s, NICE, 1.6, 0.1);
    glPopMatrix();
}

void GLWindow2::DrawCaption(string s)
{
    if(s.length() == 0)
        return;

    SetupWindowOrtho();
    // Find out how many lines are in the caption:
    // Count the endls
    int nLines = 0;
    {
        string sendl("\n");
        string::size_type st=0;
        while(1)
        {
            nLines++;
            st = s.find(sendl, st);
            if(st==string::npos)
                break;
            else
                st++;
        }
    }

    int nTopOfBox = size().y - nLines * 17;

    // Draw a grey background box for the text
    glColor4f(0,0,0,0.4);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_QUADS);
    glVertex2d(-0.5, nTopOfBox);
    glVertex2d(size().x, nTopOfBox);
    glVertex2d(size().x, size().y);
    glVertex2d(-0.5, size().y);
    glEnd();

    // Draw the caption text in yellow
    glColor3f(1,1,0);
    PrintString(ImageRef(10,nTopOfBox + 13), s);
}


void GLWindow2::HandlePendingEvents()
{
    handle_events(*this);
}

void GLWindow2::on_mouse_move(GLWindow& win, CVD::ImageRef where, int state)
{
    ImageRef irMotion = where - mirLastMousePos;
    mirLastMousePos = where;

    double dSensitivity = 0.01;
    if(state & BUTTON_LEFT && ! (state & BUTTON_RIGHT))
    {
        mvMCPoseUpdate[3] -= irMotion[1] * dSensitivity;
        mvMCPoseUpdate[4] += irMotion[0] * dSensitivity;
    }
    else if(!(state & BUTTON_LEFT) && state & BUTTON_RIGHT)
    {
        mvLeftPoseUpdate[4] -= irMotion[0] * dSensitivity;
        mvLeftPoseUpdate[3] += irMotion[1] * dSensitivity;
    }
    else if(state & BUTTON_MIDDLE  || (state & BUTTON_LEFT && state & BUTTON_RIGHT))
    {
        mvLeftPoseUpdate[5] -= irMotion[0] * dSensitivity;
        mvLeftPoseUpdate[2] += irMotion[1] * dSensitivity;
    }

}

void GLWindow2::on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button)
{
    bool bHandled = false;
    for(unsigned int i=0; !bHandled && i<mvpGLWindowMenus.size(); i++)
        bHandled = mvpGLWindowMenus[i]->HandleClick(button, state, where.x, where.y);

}

void GLWindow2::on_event(GLWindow& win, int event)
{
    if(event==EVENT_CLOSE)
        GUI.ParseLine("quit");
}

pair<Vector<6>, Vector<6> > GLWindow2::GetMousePoseUpdate()
{
    pair<Vector<6>, Vector<6> > result = make_pair(mvLeftPoseUpdate, mvMCPoseUpdate);
    mvLeftPoseUpdate = Zeros;
    mvMCPoseUpdate = Zeros;
    return result;
}



#ifndef WIN32
#include <X11/keysym.h>
void GLWindow2::on_key_down(GLWindow&, int k)
{
    string s;
    switch(k)
    {
    case XK_a:   case XK_A:  s="a"; break;
    case XK_b:   case XK_B:  s="b"; break;
    case XK_c:   case XK_C:  s="c"; break;
    case XK_d:   case XK_D:  s="d"; break;
    case XK_e:   case XK_E:  s="e"; break;
    case XK_f:   case XK_F:  s="f"; break;
    case XK_g:   case XK_G:  s="g"; break;
    case XK_h:   case XK_H:  s="h"; break;
    case XK_i:   case XK_I:  s="i"; break;
    case XK_j:   case XK_J:  s="j"; break;
    case XK_k:   case XK_K:  s="k"; break;
    case XK_l:   case XK_L:  s="l"; break;
    case XK_m:   case XK_M:  s="m"; break;
    case XK_n:   case XK_N:  s="n"; break;
    case XK_o:   case XK_O:  s="o"; break;
    case XK_p:   case XK_P:  s="p"; break;
    case XK_q:   case XK_Q:  s="q"; break;
    case XK_r:   case XK_R:  s="r"; break;
    case XK_s:   case XK_S:  s="s"; break;
    case XK_t:   case XK_T:  s="t"; break;
    case XK_u:   case XK_U:  s="u"; break;
    case XK_v:   case XK_V:  s="v"; break;
    case XK_w:   case XK_W:  s="w"; break;
    case XK_x:   case XK_X:  s="x"; break;
    case XK_y:   case XK_Y:  s="y"; break;
    case XK_z:   case XK_Z:  s="z"; break;
    case XK_1:   s="1"; break;
    case XK_2:   s="2"; break;
    case XK_3:   s="3"; break;
    case XK_4:   s="4"; break;
    case XK_5:   s="5"; break;
    case XK_6:   s="6"; break;
    case XK_7:   s="7"; break;
    case XK_8:   s="8"; break;
    case XK_9:   s="9"; break;
    case XK_0:   s="0"; break;
    case XK_KP_Prior: case XK_Page_Up:     s="PageUp"; break;
    case XK_KP_Next:  case XK_Page_Down:   s="PageDown"; break;
    case XK_Return: s="Enter"; break;
    case XK_space:  s="Space"; break;
    case XK_BackSpace:  s="BackSpace"; break;
    case XK_Escape:  s="Escape"; break;
    default: ;
    }

    if(s!="")
        GUI.ParseLine("try KeyPress "+s);
}
#else
void GLWindow2::on_key_down(GLWindow&, int k)
{
    string s;
    // ASCI chars can be mapped directly:
    if( (k >= 48 && k <=57) || ( k >=97 && k <= 122) || (k >= 65 && k <= 90))
    {
        char c = k;
        if(c >= 65 && c <= 90)
            c = c + 32;
        s = c;
    }
    else switch (k) // Some special chars are translated:
    {
    case 33: s="PageUp"; break;
    case 34: s="PageDown"; break;
    case 13: s="Enter"; break;
    case 32:  s="Space"; break;
    case 8:  s="BackSpace"; break;
    case 27:  s="Escape"; break;
    default: break;
    }

    if(s!="")
        GUI.ParseLine("try KeyPress "+s);
}
#endif

