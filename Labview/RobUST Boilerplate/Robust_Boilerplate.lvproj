<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="24008000">
	<Property Name="varPersistentID:{00611030-CAAC-41C4-9549-466DE46B5AD8}" Type="Ref">/NI-PXIe8880-03117F0A/PXI_Host_Shared_Vars.lvlib/Tension_Telemetry</Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="IOScan.Faults" Type="Str"></Property>
		<Property Name="IOScan.NetVarPeriod" Type="UInt">100</Property>
		<Property Name="IOScan.NetWatchdogEnabled" Type="Bool">false</Property>
		<Property Name="IOScan.Period" Type="UInt">10000</Property>
		<Property Name="IOScan.PowerupMode" Type="UInt">0</Property>
		<Property Name="IOScan.Priority" Type="UInt">9</Property>
		<Property Name="IOScan.ReportModeConflict" Type="Bool">true</Property>
		<Property Name="IOScan.StartEngineOnDeploy" Type="Bool">false</Property>
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Host_Panel.vi" Type="VI" URL="../Host_Panel.vi"/>
		<Item Name="TensionChart.ctl" Type="VI" URL="../TensionChart.ctl"/>
		<Item Name="Dependencies" Type="Dependencies"/>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
	<Item Name="NI-PXIe8880-03117F0A" Type="RT PXI Chassis">
		<Property Name="alias.name" Type="Str">NI-PXIe8880-03117F0A</Property>
		<Property Name="alias.value" Type="Str">10.0.0.62</Property>
		<Property Name="CCSymbols" Type="Str">TARGET_TYPE,RT;OS,Linux;CPU,x64;</Property>
		<Property Name="host.ResponsivenessCheckEnabled" Type="Bool">true</Property>
		<Property Name="host.ResponsivenessCheckPingDelay" Type="UInt">5000</Property>
		<Property Name="host.ResponsivenessCheckPingTimeout" Type="UInt">1000</Property>
		<Property Name="host.TargetCPUID" Type="UInt">9</Property>
		<Property Name="host.TargetOSID" Type="UInt">19</Property>
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="target.cleanupVisa" Type="Bool">false</Property>
		<Property Name="target.FPProtocolGlobals_ControlTimeLimit" Type="Int">300</Property>
		<Property Name="target.getDefault-&gt;WebServer.Port" Type="Int">80</Property>
		<Property Name="target.getDefault-&gt;WebServer.Timeout" Type="Int">60</Property>
		<Property Name="target.IOScan.Faults" Type="Str"></Property>
		<Property Name="target.IOScan.NetVarPeriod" Type="UInt">100</Property>
		<Property Name="target.IOScan.NetWatchdogEnabled" Type="Bool">false</Property>
		<Property Name="target.IOScan.Period" Type="UInt">10000</Property>
		<Property Name="target.IOScan.PowerupMode" Type="UInt">0</Property>
		<Property Name="target.IOScan.Priority" Type="UInt">0</Property>
		<Property Name="target.IOScan.ReportModeConflict" Type="Bool">true</Property>
		<Property Name="target.IsRemotePanelSupported" Type="Bool">true</Property>
		<Property Name="target.RTCPULoadMonitoringEnabled" Type="Bool">true</Property>
		<Property Name="target.RTDebugWebServerHTTPPort" Type="Int">8001</Property>
		<Property Name="target.RTTarget.ApplicationPath" Type="Path">/c/ni-rt/startup/startup.rtexe</Property>
		<Property Name="target.RTTarget.EnableFileSharing" Type="Bool">true</Property>
		<Property Name="target.RTTarget.IPAccess" Type="Str">+*</Property>
		<Property Name="target.RTTarget.LaunchAppAtBoot" Type="Bool">false</Property>
		<Property Name="target.RTTarget.VIPath" Type="Path">/home/lvuser/natinst/bin</Property>
		<Property Name="target.server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.server.tcp.access" Type="Str">+*</Property>
		<Property Name="target.server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="target.server.tcp.paranoid" Type="Bool">true</Property>
		<Property Name="target.server.tcp.port" Type="Int">3363</Property>
		<Property Name="target.server.tcp.serviceName" Type="Str">Main Application Instance/VI Server</Property>
		<Property Name="target.server.tcp.serviceName.default" Type="Str">Main Application Instance/VI Server</Property>
		<Property Name="target.server.vi.access" Type="Str">+*</Property>
		<Property Name="target.server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="target.server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.WebServer.Config" Type="Str">Listen 8000

NI.ServerName default
DocumentRoot "$LVSERVER_DOCROOT"
TypesConfig "$LVSERVER_CONFIGROOT/mime.types"
DirectoryIndex index.htm
WorkerLimit 10
InactivityTimeout 60

LoadModulePath "$LVSERVER_MODULEPATHS"
LoadModule LVAuth lvauthmodule
LoadModule LVRFP lvrfpmodule

#
# Pipeline Definition
#

SetConnector netConnector

AddHandler LVAuth
AddHandler LVRFP

AddHandler fileHandler ""

AddOutputFilter chunkFilter


</Property>
		<Property Name="target.WebServer.Enabled" Type="Bool">false</Property>
		<Property Name="target.WebServer.LogEnabled" Type="Bool">false</Property>
		<Property Name="target.WebServer.LogPath" Type="Path">/c/ni-rt/system/www/www.log</Property>
		<Property Name="target.WebServer.Port" Type="Int">80</Property>
		<Property Name="target.WebServer.RootPath" Type="Path">/c/ni-rt/system/www</Property>
		<Property Name="target.WebServer.TcpAccess" Type="Str">c+*</Property>
		<Property Name="target.WebServer.Timeout" Type="Int">60</Property>
		<Property Name="target.WebServer.ViAccess" Type="Str">+*</Property>
		<Property Name="target.webservices.SecurityAPIKey" Type="Str">PqVr/ifkAQh+lVrdPIykXlFvg12GhhQFR8H9cUhphgg=:pTe9HRlQuMfJxAG6QCGq7UvoUpJzAzWGKy5SbZ+roSU=</Property>
		<Property Name="target.webservices.ValidTimestampWindow" Type="Int">15</Property>
		<Item Name="CAN" Type="Folder">
			<Item Name="CAN_close_all.vi" Type="VI" URL="../CAN_close_all.vi"/>
			<Item Name="CAN_enable_MAGIC.vi" Type="VI" URL="../CAN_enable_MAGIC.vi"/>
			<Item Name="CAN_example_darren.vi" Type="VI" URL="../CAN_example_darren.vi"/>
			<Item Name="CAN_initialize_all.vi" Type="VI" URL="../CAN_initialize_all.vi"/>
			<Item Name="CAN_write_current.vi" Type="VI" URL="../CAN_write_current.vi"/>
		</Item>
		<Item Name="Load Cells" Type="Folder">
			<Item Name="Read_Load_Cell_Tensions.vi" Type="VI" URL="../Read_Load_Cell_Tensions.vi"/>
		</Item>
		<Item Name="Utility" Type="Folder">
			<Item Name="clamp_vals_array.vi" Type="VI" URL="../clamp_vals_array.vi"/>
			<Item Name="Goal_Tensions_FGV.vi" Type="VI" URL="../Goal_Tensions_FGV.vi"/>
		</Item>
		<Item Name="Controllers" Type="Folder">
			<Item Name="Manual_Control_Main.vi" Type="VI" URL="../Manual_Control_Main.vi"/>
			<Item Name="TCP_Main.vi" Type="VI" URL="../TCP_Main.vi"/>
		</Item>
		<Item Name="CL_control.vi" Type="VI" URL="../CL_control.vi"/>
		<Item Name="OL_control.vi" Type="VI" URL="../OL_control.vi"/>
		<Item Name="PXI_Host_Shared_Vars.lvlib" Type="Library" URL="../PXI_Host_Shared_Vars.lvlib"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="CANopen BaudRate.ctl" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen BaudRate.ctl"/>
				<Item Name="CANopen CAN Frame Read Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen CAN Frame Read Close.vi"/>
				<Item Name="CANopen CAN Frame Read Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen CAN Frame Read Create.vi"/>
				<Item Name="CANopen CAN Frame Read Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen CAN Frame Read Start.vi"/>
				<Item Name="CANopen Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Close.vi"/>
				<Item Name="CANopen Convert To Data [DBL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [DBL].vi"/>
				<Item Name="CANopen Convert To Data [I8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [I8].vi"/>
				<Item Name="CANopen Convert To Data [I16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [I16].vi"/>
				<Item Name="CANopen Convert To Data [I32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [I32].vi"/>
				<Item Name="CANopen Convert To Data [SGL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [SGL].vi"/>
				<Item Name="CANopen Convert To Data [U8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [U8].vi"/>
				<Item Name="CANopen Convert To Data [U16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [U16].vi"/>
				<Item Name="CANopen Convert To Data [U32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Convert To Data [U32].vi"/>
				<Item Name="CANopen Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Create.vi"/>
				<Item Name="CANopen Emergency Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Emergency Close.vi"/>
				<Item Name="CANopen Emergency Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Emergency Create.vi"/>
				<Item Name="CANopen Emergency Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Emergency Read.vi"/>
				<Item Name="CANopen Emergency Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Emergency Start.vi"/>
				<Item Name="CANopen Error Control Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Error Control Read.vi"/>
				<Item Name="CANopen Fetch from Data [DBL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [DBL].vi"/>
				<Item Name="CANopen Fetch from Data [I8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [I8].vi"/>
				<Item Name="CANopen Fetch from Data [I16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [I16].vi"/>
				<Item Name="CANopen Fetch from Data [I32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [I32].vi"/>
				<Item Name="CANopen Fetch from Data [SGL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [SGL].vi"/>
				<Item Name="CANopen Fetch from Data [STR].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [STR].vi"/>
				<Item Name="CANopen Fetch from Data [U8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [U8].vi"/>
				<Item Name="CANopen Fetch from Data [U16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [U16].vi"/>
				<Item Name="CANopen Fetch from Data [U32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data [U32].vi"/>
				<Item Name="CANopen Fetch from Data.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Fetch from Data.vi"/>
				<Item Name="CANopen Heartbeat Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Heartbeat Close.vi"/>
				<Item Name="CANopen Heartbeat Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Heartbeat Create.vi"/>
				<Item Name="CANopen Heartbeat Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Heartbeat Start.vi"/>
				<Item Name="CANopen Heartbeat State Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Heartbeat State Read.vi"/>
				<Item Name="CANopen Interface Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Interface Close.vi"/>
				<Item Name="CANopen Interface Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Interface Create.vi"/>
				<Item Name="CANopen NMT Write.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen NMT Write.vi"/>
				<Item Name="CANopen Node Guarding Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Node Guarding Close.vi"/>
				<Item Name="CANopen Node Guarding Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Node Guarding Create.vi"/>
				<Item Name="CANopen Node Guarding Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Node Guarding Start.vi"/>
				<Item Name="CANopen Node Guarding State Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Node Guarding State Read.vi"/>
				<Item Name="CANopen RPDO Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen RPDO Close.vi"/>
				<Item Name="CANopen RPDO Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen RPDO Create.vi"/>
				<Item Name="CANopen RPDO Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen RPDO Start.vi"/>
				<Item Name="CANopen RPDO Write.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen RPDO Write.vi"/>
				<Item Name="CANopen SDO Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Close.vi"/>
				<Item Name="CANopen SDO Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Create.vi"/>
				<Item Name="CANopen SDO Read [Block].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [Block].vi"/>
				<Item Name="CANopen SDO Read [DBL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [DBL].vi"/>
				<Item Name="CANopen SDO Read [I8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [I8].vi"/>
				<Item Name="CANopen SDO Read [I16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [I16].vi"/>
				<Item Name="CANopen SDO Read [I32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [I32].vi"/>
				<Item Name="CANopen SDO Read [SGL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [SGL].vi"/>
				<Item Name="CANopen SDO Read [STR].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [STR].vi"/>
				<Item Name="CANopen SDO Read [U8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [U8].vi"/>
				<Item Name="CANopen SDO Read [U16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [U16].vi"/>
				<Item Name="CANopen SDO Read [U32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read [U32].vi"/>
				<Item Name="CANopen SDO Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Read.vi"/>
				<Item Name="CANopen SDO Write [Block].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [Block].vi"/>
				<Item Name="CANopen SDO Write [DBL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [DBL].vi"/>
				<Item Name="CANopen SDO Write [I8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [I8].vi"/>
				<Item Name="CANopen SDO Write [I16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [I16].vi"/>
				<Item Name="CANopen SDO Write [I32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [I32].vi"/>
				<Item Name="CANopen SDO Write [SGL].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [SGL].vi"/>
				<Item Name="CANopen SDO Write [STR].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [STR].vi"/>
				<Item Name="CANopen SDO Write [U8].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [U8].vi"/>
				<Item Name="CANopen SDO Write [U16].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [U16].vi"/>
				<Item Name="CANopen SDO Write [U32].vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write [U32].vi"/>
				<Item Name="CANopen SDO Write.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SDO Write.vi"/>
				<Item Name="CANopen Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen Start.vi"/>
				<Item Name="CANopen SYNC Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SYNC Close.vi"/>
				<Item Name="CANopen SYNC Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SYNC Create.vi"/>
				<Item Name="CANopen SYNC Send.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SYNC Send.vi"/>
				<Item Name="CANopen SYNC Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen SYNC Start.vi"/>
				<Item Name="CANopen TPDO Close.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen TPDO Close.vi"/>
				<Item Name="CANopen TPDO Create.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen TPDO Create.vi"/>
				<Item Name="CANopen TPDO Read.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen TPDO Read.vi"/>
				<Item Name="CANopen TPDO Start.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopen TPDO Start.vi"/>
				<Item Name="CANopenStatusToError.vi" Type="VI" URL="/&lt;vilib&gt;/CANopenLib/CANOPEN.llb/CANopenStatusToError.vi"/>
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="DAQmx Clear Task.vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/configure/task.llb/DAQmx Clear Task.vi"/>
				<Item Name="DAQmx Fill In Error Info.vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/miscellaneous.llb/DAQmx Fill In Error Info.vi"/>
				<Item Name="DAQmx Read (Analog 1D DBL 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 1D DBL 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 1D DBL NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 1D DBL NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Analog 1D Wfm NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 1D Wfm NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Analog 1D Wfm NChan NSamp Duration).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 1D Wfm NChan NSamp Duration).vi"/>
				<Item Name="DAQmx Read (Analog 1D Wfm NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 1D Wfm NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 2D DBL NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 2D DBL NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 2D I16 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 2D I16 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 2D I32 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 2D I32 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 2D U16 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 2D U16 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog 2D U32 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog 2D U32 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Analog DBL 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog DBL 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Analog Wfm 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog Wfm 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Analog Wfm 1Chan NSamp Duration).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog Wfm 1Chan NSamp Duration).vi"/>
				<Item Name="DAQmx Read (Analog Wfm 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Analog Wfm 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D DBL 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D DBL 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D DBL NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D DBL NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Counter 1D Pulse Freq 1 Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D Pulse Freq 1 Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D Pulse Ticks 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D Pulse Ticks 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D Pulse Time 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D Pulse Time 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D U32 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D U32 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 1D U32 NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 1D U32 NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Counter 2D DBL NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 2D DBL NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter 2D U32 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter 2D U32 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Counter DBL 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter DBL 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Counter Pulse Freq 1 Chan 1 Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter Pulse Freq 1 Chan 1 Samp).vi"/>
				<Item Name="DAQmx Read (Counter Pulse Ticks 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter Pulse Ticks 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Counter Pulse Time 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter Pulse Time 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Counter U32 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Counter U32 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D Bool 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D Bool 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D Bool NChan 1Samp 1Line).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D Bool NChan 1Samp 1Line).vi"/>
				<Item Name="DAQmx Read (Digital 1D U8 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U8 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 1D U8 NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U8 NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D U16 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U16 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 1D U16 NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U16 NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D U32 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U32 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 1D U32 NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D U32 NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D Wfm NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D Wfm NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital 1D Wfm NChan NSamp Duration).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D Wfm NChan NSamp Duration).vi"/>
				<Item Name="DAQmx Read (Digital 1D Wfm NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 1D Wfm NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 2D Bool NChan 1Samp NLine).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 2D Bool NChan 1Samp NLine).vi"/>
				<Item Name="DAQmx Read (Digital 2D U8 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 2D U8 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 2D U16 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 2D U16 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital 2D U32 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital 2D U32 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Digital Bool 1Line 1Point).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital Bool 1Line 1Point).vi"/>
				<Item Name="DAQmx Read (Digital U8 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital U8 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital U16 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital U16 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital U32 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital U32 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital Wfm 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital Wfm 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Digital Wfm 1Chan NSamp Duration).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital Wfm 1Chan NSamp Duration).vi"/>
				<Item Name="DAQmx Read (Digital Wfm 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Digital Wfm 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Power 1D DBL 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 1D DBL 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Power 1D DBL NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 1D DBL NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Power 1D Wfm NChan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 1D Wfm NChan 1Samp).vi"/>
				<Item Name="DAQmx Read (Power 1D Wfm NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 1D Wfm NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Power 2D DBL NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 2D DBL NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Power 2D I16 NChan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power 2D I16 NChan NSamp).vi"/>
				<Item Name="DAQmx Read (Power DBL 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power DBL 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Power Wfm 1Chan 1Samp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power Wfm 1Chan 1Samp).vi"/>
				<Item Name="DAQmx Read (Power Wfm 1Chan NSamp).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Power Wfm 1Chan NSamp).vi"/>
				<Item Name="DAQmx Read (Raw 1D I8).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D I8).vi"/>
				<Item Name="DAQmx Read (Raw 1D I16).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D I16).vi"/>
				<Item Name="DAQmx Read (Raw 1D I32).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D I32).vi"/>
				<Item Name="DAQmx Read (Raw 1D U8).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D U8).vi"/>
				<Item Name="DAQmx Read (Raw 1D U16).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D U16).vi"/>
				<Item Name="DAQmx Read (Raw 1D U32).vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read (Raw 1D U32).vi"/>
				<Item Name="DAQmx Read.vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/read.llb/DAQmx Read.vi"/>
				<Item Name="DAQmx Start Task.vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/configure/task.llb/DAQmx Start Task.vi"/>
				<Item Name="DAQmx Stop Task.vi" Type="VI" URL="/&lt;vilib&gt;/DAQmx/configure/task.llb/DAQmx Stop Task.vi"/>
				<Item Name="EPOS2 Initialize_True.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/EPOS2 Initialize_True.vi"/>
				<Item Name="EPOS2 Library.lvlib" Type="Library" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/EPOS2 Library.lvlib"/>
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="NI_PtbyPt.lvlib" Type="Library" URL="/&lt;vilib&gt;/ptbypt/NI_PtbyPt.lvlib"/>
				<Item Name="sbCANopenBaseClass.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/sbCANopenBaseClass.lvclass"/>
				<Item Name="sdCANopenCreateCOB-IDMethod.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenCreateCOB-IDMethod.ctl"/>
				<Item Name="sdCANopenHeartbeatCommand.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenHeartbeatCommand.ctl"/>
				<Item Name="sdCANopenIndCom.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/CANopen IndCom Interface/sdCANopenIndCom.lvclass"/>
				<Item Name="sdCANopenNMTCommand.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenNMTCommand.ctl"/>
				<Item Name="sdCANopenNodeIDtoCOB-ID.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/SubVIs/sdCANopenNodeIDtoCOB-ID.vi"/>
				<Item Name="sdCANopenNodeStatus.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenNodeStatus.ctl"/>
				<Item Name="sdCANopenPDOWriteData.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenPDOWriteData.ctl"/>
				<Item Name="sdCANopenResetNode.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/sdCANopenResetNode.vi"/>
				<Item Name="sdCANopenSDOReadData.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenSDOReadData.ctl"/>
				<Item Name="sdCANopenSDOWriteData.ctl" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CANopen/TypeDefs/sdCANopenSDOWriteData.ctl"/>
				<Item Name="sdCANutility4xU8toI32.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CAN/Utility/sdCANutility4xU8toI32.vi"/>
				<Item Name="sdCANutility4xU8toU32.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CAN/Utility/sdCANutility4xU8toU32.vi"/>
				<Item Name="sdCANutilityI32to4xU8.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CAN/Utility/sdCANutilityI32to4xU8.vi"/>
				<Item Name="sdCANutilityU32to4xU8.vi" Type="VI" URL="/&lt;vilib&gt;/addons/Maxon Motor/EPOS2 CANopen/CAN/Utility/sdCANutilityU32to4xU8.vi"/>
				<Item Name="NI_PID__prctrl compat.lvlib" Type="Library" URL="/&lt;vilib&gt;/addons/control/pid/NI_PID__prctrl compat.lvlib"/>
				<Item Name="NI_PID_pid.lvlib" Type="Library" URL="/&lt;vilib&gt;/addons/control/pid/NI_PID_pid.lvlib"/>
				<Item Name="lvpidtkt.dll" Type="Document" URL="/&lt;vilib&gt;/Addons/control/pid/lvpidtkt.dll"/>
				<Item Name="TCP Listen List Operations.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/tcp.llb/TCP Listen List Operations.ctl"/>
				<Item Name="TCP Listen Internal List.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tcp.llb/TCP Listen Internal List.vi"/>
				<Item Name="Internecine Avoider.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tcp.llb/Internecine Avoider.vi"/>
				<Item Name="TCP Listen.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tcp.llb/TCP Listen.vi"/>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="/&lt;resource&gt;/lvanlys.dll"/>
			<Item Name="nicanopenlvapi.dll" Type="Document" URL="nicanopenlvapi.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="nilvaiu.dll" Type="Document" URL="nilvaiu.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
