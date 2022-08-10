import wx
import wx.lib.agw.rulerctrl, wx.lib.intctrl, wx.lib.masked.ipaddrctrl, wx.adv
import sys

import io, rpc, serial, serial.tools.list_ports, socket, time
#import numpy as np
import random
import json
import threading
import os
import pkg_resources.py2_warn

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/') + '/'

class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.out = aWxTextCtrl

    def write(self,string):
        self.out.WriteText(string)

class MyPanel(wx.Panel):

    def __init__(self,parent=None):
        wx.Panel.__init__(self,parent,id=-1)
        self.parent = parent
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.bitmap1 = None
        self.bitmap2 = None

    def action(self):
        self.bitmap1 = None
        self.bitmap2 = None
        t = threading.Thread(target=self.parent.camera_monitoring, args=(self,))
        t.setDaemon(True)
        t.start()

    def OnPaint(self, evt):
        if self.bitmap1 != None and self.bitmap2 != None:
            dc = wx.BufferedPaintDC(self)
            dc.Clear()
            dc.DrawBitmap(self.bitmap1, 1,1)
            dc.DrawBitmap(self.bitmap2, 482,1)
        else:
            self.parent.fail_counter += 1
            if self.parent.fail_counter > 10:
                self.parent.connction_stop_event.clear()
                self.parent.start_camera_button.Enable()


class Threshold_Tuner(wx.Frame):

    def __init__(self, *args, **kw):
        super(Threshold_Tuner, self).__init__(*args, **kw)
        self.image_data = bytearray()
        self.threshold_Dict ={'demo':{"th":[0,100,-127,127,-127,127],"pixel":200,"area":200}}
        self.threshold_file_is_loaded = False
        self.slider_event1 = threading.Event()
        self.connction_stop_event = threading.Event()
        self.connction_stop_event.set()
        self.new_timer = 0
        self.thresholds_are_changing = False
        self.panel = wx.Panel(self)
        self.filename = ''
        self.current_device = 'demo'
        with open(current_work_directory + "Threshold_Tuner_config.json", "r") as f:
                self.config = json.loads(f.read())
        self.COM_port = self.config['COM_port']
        self.USB_as_connection = self.config['USB_as_connection']
        self.host_IP = self.config['host_IP']
        self.remote_IP = self.config['remote_IP']
        self.defaultFile = self.config['defaultFile']
        self.threshold_Dict['demo'] = self.config['demo']
        #self.COM_port = 'COM4'
        #self.USB_as_connection = True
        #self.host_IP = '192.168.001.008'
        #self.remote_IP = '192.168.001.016'
        #self.defaultFile = ''
        self.InitUI()
        self.fail_counter = 0
        img = wx.Image(640, 480)
        self.interface = None
        self.blob_detection = 0
        self.blobs_are_changing = False
        
        


    def InitUI(self):

        self.console_Panel = wx.Panel(self)
        self.log = wx.TextCtrl(self.console_Panel, -1, style=wx.TE_MULTILINE) #|wx.TE_READONLY) #|wx.HSCROLL)
        log_box = wx.BoxSizer(wx.VERTICAL)
        log_box.Add(self.log, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP)
        self.console_Panel.SetSizer(log_box)
        redir = RedirectText(self.log)
        sys.stdout = redir
        #sys.stderr = redir
        self.CreateMenuBar()
        self.videopanel = MyPanel(self)
        self.videopanel.SetBackgroundStyle(wx.BG_STYLE_SYSTEM)
        self.videopanel.SetBackgroundColour(wx.BLACK)
        self.videopanel.SetMinSize((963,362))
        pnl2 = wx.Panel(self)
        pnl2.SetBackgroundStyle(wx.BG_STYLE_SYSTEM)

        #self.size_of_image = (480, 360)
        #self.image = wx.Image(self.size_of_image[0],self.size_of_image[1])
        #self.imageBit = wx.Bitmap(self.image)
        #self.videopanel.bitmap1 = wx.StaticBitmap(self.videopanel, wx.ID_ANY, self.imageBit, pos=(10, 1))
        #self.videopanel.bitmap2 = wx.StaticBitmap(self.videopanel, wx.ID_ANY, self.imageBit, pos=(500, 1))

        self.quit_button =  wx.Button(self.panel, wx.ID_ANY, "Quit")
        self.save_and_exit_button = wx.Button(self.panel, wx.ID_ANY, "Save&Exit")
        self.start_camera_button = wx.Button(self.panel, wx.ID_ANY, "Start Camera")
        self.reset_LAB_button = wx.Button(self.panel, wx.ID_ANY, "Reset LAB")
        self.load_file_button = wx.Button(self.panel, wx.ID_ANY, "Load File")
        self.device_selector = wx.ComboBox(self.panel, value='demo',size=(120,25),
                                          choices=['demo','','','','','','','','',''], style=wx.CB_DROPDOWN)
        self.pixel_text = wx.StaticText(self.panel, label='Pixel TH',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.area_text = wx.StaticText(self.panel, label='Area TH',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.pixel_input = wx.lib.intctrl.IntCtrl(self.panel, value=self.threshold_Dict[self.current_device]['pixel'],
                                                 size=(60,25), min=1, max=20000)
        self.area_input = wx.lib.intctrl.IntCtrl(self.panel, value=self.threshold_Dict[self.current_device]['area'],
                                                size=(60,25), min=1, max=20000)

        #self.device_selector = wx.ComboBox()
        val1, val2, val3, val4, val5, val6 = self.threshold_Dict['demo']['th']
        self.Lmin = wx.Slider(pnl2, value=val1, minValue=0, maxValue=100, style = wx.SL_LABELS|wx.SL_AUTOTICKS, name = 'L min')
        self.Lmax = wx.Slider(pnl2, value=val2, minValue=0, maxValue=100, style = wx.SL_TOP|wx.SL_LABELS|wx.SL_AUTOTICKS)
        self.Amin = wx.Slider(pnl2, value=val3, minValue=-127, maxValue=127, style = wx.SL_LABELS|wx.SL_AUTOTICKS)
        self.Amax = wx.Slider(pnl2, value=val4, minValue=-127, maxValue=127, style = wx.SL_TOP|wx.SL_LABELS|wx.SL_AUTOTICKS)
        self.Bmin = wx.Slider(pnl2, value=val5, minValue=-127, maxValue=127, style = wx.SL_LABELS|wx.SL_AUTOTICKS)
        self.Bmax = wx.Slider(pnl2, value=val6, minValue=-127, maxValue=127, style = wx.SL_TOP|wx.SL_LABELS|wx.SL_AUTOTICKS)
        self.Lmin.SetTickFreq(10)
        self.Amin.SetTickFreq(10)
        self.Bmin.SetTickFreq(10)
        self.text1 = wx.StaticText(pnl2, label='L min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text11 = wx.StaticText(pnl2, label='L min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text2 = wx.StaticText(pnl2, label='L max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text21 = wx.StaticText(pnl2, label='L max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text3 = wx.StaticText(pnl2, label='A min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text31 = wx.StaticText(pnl2, label='A min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text4 = wx.StaticText(pnl2, label='A max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text41 = wx.StaticText(pnl2, label='A max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text5 = wx.StaticText(pnl2, label='B min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text51 = wx.StaticText(pnl2, label='B min',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text6 = wx.StaticText(pnl2, label='B max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.text61 = wx.StaticText(pnl2, label='B max',size=(60,25), style= wx.ALIGN_CENTRE_HORIZONTAL)
        #self.text1.SetLabelMarkup("<b>L min</b>")


        self.button_box = wx.BoxSizer(wx.HORIZONTAL)
        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        hbox3 = wx.BoxSizer(wx.HORIZONTAL)
        hbox4 = wx.BoxSizer(wx.HORIZONTAL)
        hbox5 = wx.BoxSizer(wx.HORIZONTAL)
        hbox6 = wx.BoxSizer(wx.HORIZONTAL)

        self.button_box.Add(self.quit_button, proportion=0)
        self.button_box.Add(self.save_and_exit_button, proportion=0)
        self.button_box.Add(self.start_camera_button, proportion=0)
        self.button_box.Add(self.reset_LAB_button, proportion=0)
        self.button_box.Add(self.load_file_button, proportion=0)
        self.button_box.Add(self.device_selector, proportion=0, flag=wx.RIGHT )
        self.button_box.Add(self.pixel_text, proportion=0, flag=wx.RIGHT )
        self.button_box.Add(self.pixel_input, proportion=0, flag=wx.RIGHT )
        self.button_box.Add(self.area_text, proportion=0, flag=wx.RIGHT )
        self.button_box.Add(self.area_input, proportion=0, flag=wx.RIGHT )

        hbox1.Add(self.text1, proportion=0)
        hbox1.Add(self.Lmin, proportion=1)
        hbox1.Add(self.text11, proportion=0)
        hbox2.Add(self.text2, proportion=0)
        hbox2.Add(self.Lmax, proportion=1)
        hbox2.Add(self.text21, proportion=0)
        hbox3.Add(self.text3, proportion=0)
        hbox3.Add(self.Amin, proportion=1)
        hbox3.Add(self.text31, proportion=0)
        hbox4.Add(self.text4, proportion=0)
        hbox4.Add(self.Amax, proportion=1)
        hbox4.Add(self.text41, proportion=0)
        hbox5.Add(self.text5, proportion=0)
        hbox5.Add(self.Bmin, proportion=1)
        hbox5.Add(self.text51, proportion=0)
        hbox6.Add(self.text6, proportion=0)
        hbox6.Add(self.Bmax, proportion=1)
        hbox6.Add(self.text61, proportion=0)

        vbox.Add(hbox1, proportion = 2, flag=wx.EXPAND|wx.BOTTOM) #, border=10)
        vbox.Add(hbox2, proportion = 2, flag=wx.EXPAND|wx.BOTTOM) #, border=10)
        vbox.Add(hbox3, proportion = 2, flag=wx.EXPAND|wx.BOTTOM) #, border=10)
        vbox.Add(hbox4, proportion = 0, flag=wx.EXPAND|wx.BOTTOM) #, border=10)
        vbox.Add(hbox5, proportion = 2, flag=wx.EXPAND|wx.BOTTOM) #, border=10)
        vbox.Add(hbox6, proportion = 2, flag=wx.EXPAND|wx.BOTTOM) #, border=10)

        self.panel.SetSizer(self.button_box)
        pnl2.SetSizer(vbox)
        self.button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.button_sizer.Add(self.panel, proportion=2, flag=wx.EXPAND  )
        image_sizer = wx.BoxSizer(wx.HORIZONTAL)
        image_sizer.Add(self.videopanel, proportion=2, flag=wx.EXPAND )

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.button_sizer, proportion=0, flag=wx.EXPAND )
        sizer.Add(image_sizer, proportion=2, flag=wx.EXPAND)
        sizer.Add(pnl2, flag=wx.EXPAND|wx.BOTTOM|wx.TOP, border=1)

        whole_window = wx.BoxSizer(wx.HORIZONTAL)
        whole_window.Add(sizer, proportion = 0, flag=wx.EXPAND )
        whole_window.Add(self.console_Panel, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP  )

        self.SetMinSize((1250, 745))
        self.CreateStatusBar()
        self.SetSizer(whole_window)

        self.SetSize((1250, 745))
        self.SetTitle('Threshold Tuner')
        self.Centre()
        self.Bind(wx.EVT_SLIDER, self.On_Slider_move)
        self.Bind(wx.EVT_MENU, self.On_Load_File, id = 1 )
        self.Bind(wx.EVT_MENU, self.On_Save, id = 2 )
        self.Bind(wx.EVT_MENU, self.On_Save_as, id = 3 )
        self.Bind(wx.EVT_MENU, self.On_Quit_select, id = 5 )
        self.Bind(wx.EVT_MENU, self.On_Deafault_Connection, id = 6 )
        self.Bind(wx.EVT_MENU, self.On_USB_input, id = 7 )
        self.Bind(wx.EVT_MENU, self.On_WIFI_input, id = 8 )
        self.Bind(wx.EVT_MENU_RANGE, self.On_Blobs_Change, id = 10, id2 = 15 )
        self.Bind(wx.EVT_MENU, self.OnAbout, id = 21 )
        self.Bind(wx.EVT_MENU, self.Quick_Start, id = 22 )
        #self.Bind(wx.EVT_MENU, self.On_Start_Camera, id = 3 )
        self.quit_button.Bind(wx.EVT_BUTTON, self.On_Quit_select)
        self.start_camera_button.Bind(wx.EVT_BUTTON, self.On_Start_Camera )
        self.reset_LAB_button.Bind(wx.EVT_BUTTON, self.On_Reset_LAB )
        self.load_file_button.Bind(wx.EVT_BUTTON, self.On_Load_File)
        self.save_and_exit_button.Bind(wx.EVT_BUTTON, self.On_Save_and_Exit)
        self.device_selector.Bind(wx.EVT_COMBOBOX, self.On_device_selector)
        self.pixel_input.Bind(wx.EVT_TEXT, self.On_number_input)
        self.area_input.Bind(wx.EVT_TEXT, self.On_number_input)

    def Quick_Start(self, event):
        print('1. Verify that remote part of this software is launched on OpenMV cam',
              '\n2. Load thresholds from file by pressing button <Load File> or through menu item <File>/<Load from file>.',
              ' Note that it is not allowed to load file 2 times in single session.',
              ' You have to restart program if you need to tune thresholds in different file.',
              '\n3. Choose communication channel through menu item <File>/<Connection Port> and set up settings or just use <Default>',
              ' if you have it done last time.',
              '\n4. Press button <Start Camera>',
              '\n5. Choose color device from dropdown menu under "demo" and tune thresholds by sliders',
              '\n6. Note that moving sliders will affect to changing of thresholds after sliders stops moving'
              '\n7. press <Reset LAB> if you wish to lead all sliders quickly to position with maximum gaps',
              '\n8. Blobs detection mode can be chosen under <Blobs> menu',
              '\n9. <Pixel TH> and <Area TH> settings affect to detection of blobs.',
              ' With bigger values in these settings less blobs can be detected',
              '\n10. Save changes into loaded thresholds file or save in new file by designation new filename in menu item <File>/<Save as>',
              '\n11. Press <Quit> button if you wish to exit without saving.',
              '\n USB settings, WiFi setting, default file name and current slider positions for "demo" device',
              ' will be stored in configuration file for next session')

    def OnAbout(self, event):
        aboutInfo = wx.adv.AboutDialogInfo()
        aboutInfo.SetName("Threshold Tuner")
        aboutInfo.SetVersion('Version 1.0')
        aboutInfo.SetDescription("With this app you can tune fast and convenient\n   color thresholds for OpenMV camera.")
        aboutInfo.SetCopyright("(C) 2020")
        aboutInfo.SetWebSite("www.robokit.su")
        aboutInfo.AddDeveloper("Azer Babaev")
        wx.adv.AboutBox(aboutInfo)

    def On_Deafault_Connection(self, event):
        with open(current_work_directory + "Threshold_Tuner_config.json", "r") as f:
                config = json.loads(f.read())
        self.COM_port = config['COM_port']
        self.USB_as_connection = config['USB_as_connection']
        self.host_IP = config['host_IP']
        self.remote_IP = config['remote_IP']


    def On_WIFI_input(self, event):
        self.USB_as_connection = False
        wifi_input = wx.Dialog(self, title='WiFi input')
        dialog_sizer = wx.BoxSizer(wx.VERTICAL)
        button_sizer = wifi_input.CreateButtonSizer(flags =  wx.OK|wx.CANCEL)
        my_ip_text = wx.lib.masked.ipaddrctrl.IpAddrCtrl(wifi_input, value = self.host_IP, name='Input IP address of Host Computer')
        remote_ip_text = wx.lib.masked.ipaddrctrl.IpAddrCtrl(wifi_input, value = self.remote_IP, name='Input IP address of remote OpenMV')
        text1 = wx.StaticText(wifi_input, label='Input IP address of Host Computer',size=(200,25))
        text2 = wx.StaticText(wifi_input, label='Input IP address of remote OpenMV',size=(200,25))
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        hbox3 = wx.BoxSizer(wx.HORIZONTAL)
        hbox1.Add(text1, proportion = 1, flag = wx.EXPAND , border = 10)
        hbox1.Add(my_ip_text, proportion = 1, flag = wx.EXPAND, border = 10 )
        hbox2.Add(text2, proportion = 1, flag = wx.EXPAND , border = 10)
        hbox2.Add(remote_ip_text, proportion = 1, flag = wx.EXPAND, border = 10 )
        hbox3.Add(button_sizer, proportion = 1, flag = wx.EXPAND|wx.CENTER , border = 10)
        dialog_sizer.Add(hbox1, proportion = 1, flag = wx.EXPAND|wx.CENTER, border = 10 )
        dialog_sizer.Add(hbox2 , proportion = 1, flag = wx.EXPAND|wx.CENTER, border = 10)
        dialog_sizer.Add(hbox3 , proportion = 1, flag = wx.EXPAND|wx.CENTER, border = 10)
        dialog_sizer.Fit(wifi_input)
        wifi_input.SetSizer(dialog_sizer)
        if wifi_input.ShowModal()== wx.ID_OK:
            self.host_IP = my_ip_text.GetAddress()
            self.remote_IP = remote_ip_text.GetAddress()
            print('host_IP = ', self.host_IP)
            print('remote_IP = ', self.remote_IP)
        

    def On_USB_input(self, event):
        choices = []
        message = 'List of available ports. Choose COM port'
        for port, desc, hwid in serial.tools.list_ports.comports():
            message = message + '\n' + str(port) + ' - ' + str(desc)
            choices.append(str(port))
        input_USB_dialog = wx.SingleChoiceDialog(self, message,
                                               'USB select', choices, style = wx.OK|wx.CANCEL|wx.OK_DEFAULT|wx.CENTRE)
        #input_USB_dialog.CreateTextSizer(message, 120)
        #wx.MessageBox(message, parent = input_USB_dialog)
        #USB_selector = wx.ComboBox(input_USB_dialog, value='',size=(120,25),
        #                                  choices=['','','','','','','','','',''], style=wx.CB_DROPDOWN)
        #input_USB_dialog.Add(USB_selector)
        if input_USB_dialog.ShowModal() == wx.ID_OK:
            self.COM_port = input_USB_dialog.GetStringSelection()
        self.USB_as_connection = True
        print('COM_port = ', self.COM_port )

    def On_number_input(self, event):
        if event.GetEventObject() == self.pixel_input:
            self.threshold_Dict[self.current_device]['pixel'] = self.pixel_input.GetValue()
        if event.GetEventObject() == self.area_input:
            self.threshold_Dict[self.current_device]['area'] = self.area_input.GetValue()
        self.blobs_are_changing = True
        self.slider_event1.set()

    def On_Blobs_Change(self, event):
        id = event.GetId()
        self.blob_detection = id - 10
        self.blobs_are_changing = True
        self.slider_event1.set()

    def On_Save(self, event):
        data = self.threshold_Dict.copy()
        data.pop('demo')
        with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_as(self, event):
        save_file_dialog = wx.FileDialog(None, message="Select .json file with thresholds",
                                        wildcard = '*.json', style =wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT)
        success_code = save_file_dialog.ShowModal()
        if success_code == wx.ID_OK:
            self.filename = save_file_dialog.GetPath()
            self.filename = self.filename.replace('\\', '/')
            data = self.threshold_Dict.copy()
            data.pop('demo')
            with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_and_Exit(self, event):
        self.On_Save(event)
        self.On_Quit_select(event)

    def CreateMenuBar(self):

        menubar = wx.MenuBar()
        self.filem = wx.Menu()
        #reset = wx.Menu()
        #color_devices = wx.Menu()
        help = wx.Menu()
        self.blobs = wx.Menu()

        connect = wx.Menu()
        connect.AppendRadioItem(6, 'Default', 'Connect through USB')
        connect.AppendRadioItem(7, 'USB', 'Connect through USB')
        connect.AppendRadioItem(8, 'WiFi', 'Connect through WiFi')

        self.filem.Append(1, '&Load from file', 'Load thresholds from file')
        self.filem.Append(2, '&Save', 'Save thresholds to loaded file')
        self.filem.Append(3, 'Save as', 'Save thresholds to new file')
        self.filem.AppendSubMenu(connect, '&Connection Port') #'Choose connection port to OpenMV')
        self.filem.Append(5, '&Quit', 'Quit application')

        menubar.Append(self.filem, '&File')
        #menubar.Append(reset, '&Reset LAB')
        #menubar.Append(color_devices, '&Color Devices')
        menubar.Append(self.blobs, '&Blobs')
        menubar.Append(help, '&Help')

        help.Append(21,'About')
        help.Append(22,'Quick Start')

        self.blobs.AppendRadioItem(10, 'Blobs Detection OFF', 'Blobs Detection OFF')
        self.blobs.AppendRadioItem(11, 'Blobs Detection ON', 'Blobs Detection ON')
        self.blobs.AppendRadioItem(12, 'Orange Ball on Green field', 'Blobs Detection ON')
        self.blobs.AppendRadioItem(13, 'Blue Post on Green field', 'Blobs Detection ON')
        self.blobs.AppendRadioItem(14, 'Yellow Post on Green field', 'Blobs Detection ON')
        self.blobs.AppendRadioItem(15, 'White Post on Green field', 'Blobs Detection ON')
        for i in range(12,16,1): self.blobs.Enable(i, False)


        #color_devices.AppendRadioItem(11, '&Demo', 'Training')
        #color_devices.AppendRadioItem(12, '&orange ball', 'orange ball')
        #color_devices.AppendRadioItem(13, '&blue posts', 'blue posts')
        #color_devices.AppendRadioItem(14, '&yellow posts', 'yellow posts')
        #color_devices.AppendRadioItem(15, '&white posts', 'white posts')
        #color_devices.AppendRadioItem(16, '&green field', 'green field')
        #color_devices.AppendRadioItem(17, '&white marking', 'white marking')

        self.SetMenuBar(menubar)

    def On_device_selector(self,event):
        value = self.device_selector.GetValue()
        if value != self.current_device:
            if value != '':
                self.current_device = value
                th = self.threshold_Dict[self.current_device]
                self.Lmin.SetValue(th['th'][0])
                self.Lmax.SetValue(th['th'][1])
                self.Amin.SetValue(th['th'][2])
                self.Amax.SetValue(th['th'][3])
                self.Bmin.SetValue(th['th'][4])
                self.Bmax.SetValue(th['th'][5])
                self.pixel_input.SetValue(th['pixel'])
                self.area_input.SetValue(th['area'])
                self.slider_event1.set()
            else:
                ind = list(self.threshold_Dict.keys()).index(self.current_device)
                self.device_selector.SetSelection(ind)

    def On_Slider_move(self, e):
        self.slider_event1.set()
        

    def On_Quit_select(self, e):
        self.config = {'USB_as_connection':self.USB_as_connection, 'COM_port': self.COM_port,
                      'host_IP': self.host_IP, 'remote_IP': self.remote_IP, 'demo': self.threshold_Dict['demo'],
                      'defaultFile': self.defaultFile}
        with open(current_work_directory + "Threshold_Tuner_config.json", "w") as f:
                json.dump(self.config, f)
        self.connction_stop_event.clear()
        self.connction_stop_event.wait(timeout=1)
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        sys.exit(0)

    def On_Reset_LAB(self, e):
        self.Lmin.SetValue(0)
        self.Lmax.SetValue(100)
        self.Amin.SetValue(-127)
        self.Amax.SetValue(127)
        self.Bmin.SetValue(-127)
        self.Bmax.SetValue(127)
        self.slider_event1.set()

    def On_Start_Camera(self, event):
        self.fail_counter = 0
        self.videopanel.action()
        self.start_camera_button.Disable()

    #def On_Start_Camera1(self, event):
    #    self.fail_counter = 0
    #    #self.camera_connection()
    #    t = threading.Thread(target= self.camera_monitoring) #, args=( ))
    #    t.setDaemon(True)
    #    t.start()
    #    #t1 = threading.Thread(target= self.rebuffer) #, args=( ))
    #    #t1.setDaemon(True)
    #    #t1.start()
    #    time.sleep(1)
    #    self.timex1 = wx.Timer(self)
    #    self.timex1.Start(1000./24)
    #    self.Bind(wx.EVT_TIMER, self.redraw, self.timex1)
    #    #self.timex2 = wx.Timer(self)
    #    #self.timex2.Start(1000./24)
    #    #self.Bind(wx.EVT_TIMER, self.rebuffer, self.timex2) 
    #    #btn = event.GetEventObject()
    #    self.start_camera_button.Disable()

    def On_Load_File(self, event):
        load_file_dialog = wx.FileDialog(None, message="Select .json file with thresholds", defaultFile = self.defaultFile, wildcard = '*.json')
        success_code = load_file_dialog.ShowModal()
        if success_code == wx.ID_OK:
            self.filename = load_file_dialog.GetPath()
            self.defaultFile = self.filename
            self.filename = self.filename.replace('\\', '/')
            with open(self.filename, "r") as f:
                loaded_Dict = json.loads(f.read())
            if loaded_Dict.get('orange ball') != None:
                if loaded_Dict.get('blue posts') != None:
                    if loaded_Dict.get('yellow posts') != None:
                        if loaded_Dict.get('white posts') != None:
                            if loaded_Dict.get('green field') != None:
                                if loaded_Dict.get('white marking') != None:
                                    self.threshold_file_is_loaded = True
                                    self.threshold_Dict.update(loaded_Dict)
                                    self.load_file_button.Disable()
                                    self.filem.Delete(1)
                                    for i in range(12,16,1): self.blobs.Enable(i, True) # enable combined blobs
                                    for i in range(1,len(list(self.threshold_Dict.keys()))):
                                        self.device_selector.SetString(i,list(self.threshold_Dict.keys())[i])
        print( 'threshold_file_is_loaded =', self.threshold_file_is_loaded)

    def camera_monitoring(self, parent):
        self.parent = parent
        self.camera_connection()
        def jpg_frame_buffer_cb(data):
            self.image_data = data.copy()
            img = wx.Image(io.BytesIO(self.image_data), type =wx.BITMAP_TYPE_JPEG )
            image_is_binary = True
            for i in range(10):
                x = int(random.random()*320)
                y = int(random.random()*240)
                red_intencity = img.GetRed(x,y)
                if red_intencity > 15 and red_intencity < 240: image_is_binary = False
            if image_is_binary:
                self.parent.bitmap1 = wx.Bitmap(img.Scale(480,360))
            else: 
                self.parent.bitmap2 = wx.Bitmap(img.Scale(480,360))
            self.parent.Refresh()
            if not self.connction_stop_event.is_set(): 
                try:
                    raise ValueError('oops!')
                except ValueError: time.sleep(1.5)
            if self.slider_event1.is_set():
                self.new_timer = time.perf_counter()
                self.slider_event1.clear()
                self.thresholds_are_changing = True
            if self.thresholds_are_changing or self.blobs_are_changing:
                if (time.perf_counter() - self.new_timer) > 0.02:
                    self.thresholds_are_changing = False
                    self.blobs_are_changing = False
                    lmin = self.Lmin.GetValue()
                    lmax = self.Lmax.GetValue()
                    amin = self.Amin.GetValue()
                    amax = self.Amax.GetValue()
                    bmin = self.Bmin.GetValue()
                    bmax = self.Bmax.GetValue()
                    self.threshold_Dict[self.current_device]['th'] = [lmin, lmax, amin, amax, bmin, bmax]
                    message = {"current": self.current_device, "blob": self.blob_detection}
                    message.update(self.threshold_Dict)
                    #thresholds = '{"th":('+ str(lmin) + ',' + str(lmax) + ',' + str(amin) + ',' + str(amax)\
                    #                + ',' + str(bmin) + ',' + str(bmax) + '),"blob":' + str(self.blob_detection)\
                    #                + ',"pixel":' + str(self.threshold_Dict[self.current_device]['pixel']) + ',"area":'\
                    #                + str(self.threshold_Dict[self.current_device]['area']) + '}' 
                    thresholds = str(message)
                    print('thresholds =', self.threshold_Dict[self.current_device]['th'])
                    result = self.interface.call("jpeg_image_binary_stream", thresholds)
            
        while(True):
            if not self.connction_stop_event.is_set():
                self.connction_stop_event.set()
                return
            self.parent.Refresh()
            self.new_timer = 0
            lmin, lmax, amin, amax, bmin, bmax = self.threshold_Dict[self.current_device]['th']
            #thresholds = '{"th":('+ str(lmin) + ',' + str(lmax) + ',' + str(amin) + ',' + str(amax)\
            #                        + ',' + str(bmin) + ',' + str(bmax) + '),"blob":' + str(self.blob_detection)\
            #                        + ',"pixel":' + str(self.threshold_Dict[self.current_device]['pixel']) + ',"area":'\
            #                        + str(self.threshold_Dict[self.current_device]['area']) + '}' 
            message = {"current": self.current_device, "blob": self.blob_detection}
            message.update(self.threshold_Dict)
            thresholds = str(message)
            if self.interface != None:
                try:
                    result = self.interface.call("jpeg_image_binary_stream", thresholds)
                except Exception: pass
                if result is not None:
                    # THE REMOTE DEVICE WILL START STREAMING ON SUCCESS. SO, WE NEED TO RECEIVE DATA IMMEDIATELY.
                    self.interface.stream_reader(jpg_frame_buffer_cb, queue_depth=1, read_timeout_ms=500)
            print('Connection Lost...')
            


    def camera_connection(self):
        if self.interface == None:
    # The RPC library above is installed on your OpenMV Cam and provides mutliple classes for
        # allowing your OpenMV Cam to control over USB or WIFI.

        ##############################################################
        # Choose the interface you wish to control an OpenMV Cam over.
        ##############################################################

        # Uncomment the below lines to setup your OpenMV Cam for controlling over a USB VCP.
        #
        # * port - Serial Port Name.
        #
            #print("\nAvailable Ports:\n")
            #for port, desc, hwid in serial.tools.list_ports.comports():
            #    print("{} : {} [{}]".format(port, desc, hwid))
            #sys.stdout.write("\nPlease enter a port name: ")
            ##sys.stdout.flush()
            ##interface = rpc.rpc_usb_vcp_master(port=input())
            if self.USB_as_connection:
                try:
                    self.interface = rpc.rpc_usb_vcp_master(port= self.COM_port)
                except Exception: self.interface = None
            else:
            #print("")
        #sys.stdout.flush()

        # Uncomment the below line to setup your OpenMV Cam for controlling over WiFi.
        #
        # * slave_ip - IP address to connect to.
        # * my_ip - IP address to bind to ("" to bind to all interfaces...)
        # * port - Port to route traffic to.
        #
                try:
                    self.interface = rpc.rpc_wifi_or_ethernet_master(slave_ip="192.168.1.16",
                                                                    my_ip="192.168.1.8", port=0x1DBA)
                except Exception: self.interface = None

        ##############################################################
        # Call Back Handlers
        ##############################################################


        # This will be called with the bytes() object generated by the slave device.
        

def main():
    app = wx.App()
    th = Threshold_Tuner(None)
    th.Show()
    app.MainLoop()


if __name__ == '__main__':
    main()  

