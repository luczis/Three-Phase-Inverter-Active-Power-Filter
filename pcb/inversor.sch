<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.6.3">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="pinhead" urn="urn:adsk.eagle:library:325">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="1X16" urn="urn:adsk.eagle:footprint:22297/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="15.24" y1="0.635" x2="15.875" y2="1.27" width="0.1524" layer="21"/>
<wire x1="15.875" y1="1.27" x2="17.145" y2="1.27" width="0.1524" layer="21"/>
<wire x1="17.145" y1="1.27" x2="17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="17.78" y1="0.635" x2="17.78" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="17.78" y1="-0.635" x2="17.145" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="17.145" y1="-1.27" x2="15.875" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="15.875" y1="-1.27" x2="15.24" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="12.065" y2="1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="1.27" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-0.635" x2="12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="13.335" y2="1.27" width="0.1524" layer="21"/>
<wire x1="13.335" y1="1.27" x2="14.605" y2="1.27" width="0.1524" layer="21"/>
<wire x1="14.605" y1="1.27" x2="15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="15.24" y1="0.635" x2="15.24" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-0.635" x2="14.605" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="14.605" y1="-1.27" x2="13.335" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="13.335" y1="-1.27" x2="12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-1.27" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-1.27" x2="7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="-1.27" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-1.27" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="1.27" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-0.635" x2="-10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="1.27" x2="-8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-1.27" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="-1.27" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="0.635" x2="-14.605" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-14.605" y1="1.27" x2="-13.335" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="1.27" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-0.635" x2="-13.335" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-1.27" x2="-14.605" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-14.605" y1="-1.27" x2="-15.24" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-0.635" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="-1.27" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-19.685" y1="1.27" x2="-18.415" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-18.415" y1="1.27" x2="-17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="0.635" x2="-17.78" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="-0.635" x2="-18.415" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="0.635" x2="-17.145" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-17.145" y1="1.27" x2="-15.875" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-15.875" y1="1.27" x2="-15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="0.635" x2="-15.24" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="-0.635" x2="-15.875" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-15.875" y1="-1.27" x2="-17.145" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-17.145" y1="-1.27" x2="-17.78" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-20.32" y1="0.635" x2="-20.32" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-19.685" y1="1.27" x2="-20.32" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-20.32" y1="-0.635" x2="-19.685" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-18.415" y1="-1.27" x2="-19.685" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="17.78" y1="0.635" x2="18.415" y2="1.27" width="0.1524" layer="21"/>
<wire x1="18.415" y1="1.27" x2="19.685" y2="1.27" width="0.1524" layer="21"/>
<wire x1="19.685" y1="1.27" x2="20.32" y2="0.635" width="0.1524" layer="21"/>
<wire x1="20.32" y1="0.635" x2="20.32" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-0.635" x2="19.685" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="19.685" y1="-1.27" x2="18.415" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="18.415" y1="-1.27" x2="17.78" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-19.05" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-16.51" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-13.97" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="11" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="12" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="13" x="11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="14" x="13.97" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="15" x="16.51" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="16" x="19.05" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-20.3962" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-20.32" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="16.256" y1="-0.254" x2="16.764" y2="0.254" layer="51"/>
<rectangle x1="13.716" y1="-0.254" x2="14.224" y2="0.254" layer="51"/>
<rectangle x1="11.176" y1="-0.254" x2="11.684" y2="0.254" layer="51"/>
<rectangle x1="8.636" y1="-0.254" x2="9.144" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="-9.144" y1="-0.254" x2="-8.636" y2="0.254" layer="51"/>
<rectangle x1="-11.684" y1="-0.254" x2="-11.176" y2="0.254" layer="51"/>
<rectangle x1="-14.224" y1="-0.254" x2="-13.716" y2="0.254" layer="51"/>
<rectangle x1="-16.764" y1="-0.254" x2="-16.256" y2="0.254" layer="51"/>
<rectangle x1="-19.304" y1="-0.254" x2="-18.796" y2="0.254" layer="51"/>
<rectangle x1="18.796" y1="-0.254" x2="19.304" y2="0.254" layer="51"/>
</package>
<package name="1X16/90" urn="urn:adsk.eagle:footprint:22298/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-20.32" y1="-1.905" x2="-17.78" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="-1.905" x2="-17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="0.635" x2="-20.32" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-20.32" y1="0.635" x2="-20.32" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-19.05" y1="6.985" x2="-19.05" y2="1.27" width="0.762" layer="21"/>
<wire x1="-17.78" y1="-1.905" x2="-15.24" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="-1.905" x2="-15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="0.635" x2="-17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-16.51" y1="6.985" x2="-16.51" y2="1.27" width="0.762" layer="21"/>
<wire x1="-15.24" y1="-1.905" x2="-12.7" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-1.905" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-13.97" y1="6.985" x2="-13.97" y2="1.27" width="0.762" layer="21"/>
<wire x1="-12.7" y1="-1.905" x2="-10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-1.905" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="6.985" x2="-11.43" y2="1.27" width="0.762" layer="21"/>
<wire x1="-10.16" y1="-1.905" x2="-7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-1.905" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="6.985" x2="-8.89" y2="1.27" width="0.762" layer="21"/>
<wire x1="-7.62" y1="-1.905" x2="-5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="6.985" x2="-6.35" y2="1.27" width="0.762" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="6.985" x2="-3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="6.985" x2="-1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="0" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="6.985" x2="1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="6.985" x2="3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="6.35" y1="6.985" x2="6.35" y2="1.27" width="0.762" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-1.905" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="8.89" y1="6.985" x2="8.89" y2="1.27" width="0.762" layer="21"/>
<wire x1="10.16" y1="-1.905" x2="12.7" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-1.905" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="11.43" y1="6.985" x2="11.43" y2="1.27" width="0.762" layer="21"/>
<wire x1="12.7" y1="-1.905" x2="15.24" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-1.905" x2="15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="15.24" y1="0.635" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="13.97" y1="6.985" x2="13.97" y2="1.27" width="0.762" layer="21"/>
<wire x1="15.24" y1="-1.905" x2="17.78" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="17.78" y1="-1.905" x2="17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="17.78" y1="0.635" x2="15.24" y2="0.635" width="0.1524" layer="21"/>
<wire x1="16.51" y1="6.985" x2="16.51" y2="1.27" width="0.762" layer="21"/>
<wire x1="17.78" y1="-1.905" x2="20.32" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-1.905" x2="20.32" y2="0.635" width="0.1524" layer="21"/>
<wire x1="20.32" y1="0.635" x2="17.78" y2="0.635" width="0.1524" layer="21"/>
<wire x1="19.05" y1="6.985" x2="19.05" y2="1.27" width="0.762" layer="21"/>
<pad name="1" x="-19.05" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-16.51" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-13.97" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-11.43" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-8.89" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="-6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="-3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="-1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="11" x="6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="12" x="8.89" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="13" x="11.43" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="14" x="13.97" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="15" x="16.51" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="16" x="19.05" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-20.955" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="22.225" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-19.431" y1="0.635" x2="-18.669" y2="1.143" layer="21"/>
<rectangle x1="-16.891" y1="0.635" x2="-16.129" y2="1.143" layer="21"/>
<rectangle x1="-14.351" y1="0.635" x2="-13.589" y2="1.143" layer="21"/>
<rectangle x1="-11.811" y1="0.635" x2="-11.049" y2="1.143" layer="21"/>
<rectangle x1="-9.271" y1="0.635" x2="-8.509" y2="1.143" layer="21"/>
<rectangle x1="-6.731" y1="0.635" x2="-5.969" y2="1.143" layer="21"/>
<rectangle x1="-4.191" y1="0.635" x2="-3.429" y2="1.143" layer="21"/>
<rectangle x1="-1.651" y1="0.635" x2="-0.889" y2="1.143" layer="21"/>
<rectangle x1="0.889" y1="0.635" x2="1.651" y2="1.143" layer="21"/>
<rectangle x1="3.429" y1="0.635" x2="4.191" y2="1.143" layer="21"/>
<rectangle x1="5.969" y1="0.635" x2="6.731" y2="1.143" layer="21"/>
<rectangle x1="8.509" y1="0.635" x2="9.271" y2="1.143" layer="21"/>
<rectangle x1="11.049" y1="0.635" x2="11.811" y2="1.143" layer="21"/>
<rectangle x1="13.589" y1="0.635" x2="14.351" y2="1.143" layer="21"/>
<rectangle x1="16.129" y1="0.635" x2="16.891" y2="1.143" layer="21"/>
<rectangle x1="18.669" y1="0.635" x2="19.431" y2="1.143" layer="21"/>
<rectangle x1="-19.431" y1="-2.921" x2="-18.669" y2="-1.905" layer="21"/>
<rectangle x1="-16.891" y1="-2.921" x2="-16.129" y2="-1.905" layer="21"/>
<rectangle x1="-14.351" y1="-2.921" x2="-13.589" y2="-1.905" layer="21"/>
<rectangle x1="-11.811" y1="-2.921" x2="-11.049" y2="-1.905" layer="21"/>
<rectangle x1="-9.271" y1="-2.921" x2="-8.509" y2="-1.905" layer="21"/>
<rectangle x1="-6.731" y1="-2.921" x2="-5.969" y2="-1.905" layer="21"/>
<rectangle x1="-4.191" y1="-2.921" x2="-3.429" y2="-1.905" layer="21"/>
<rectangle x1="-1.651" y1="-2.921" x2="-0.889" y2="-1.905" layer="21"/>
<rectangle x1="0.889" y1="-2.921" x2="1.651" y2="-1.905" layer="21"/>
<rectangle x1="3.429" y1="-2.921" x2="4.191" y2="-1.905" layer="21"/>
<rectangle x1="5.969" y1="-2.921" x2="6.731" y2="-1.905" layer="21"/>
<rectangle x1="8.509" y1="-2.921" x2="9.271" y2="-1.905" layer="21"/>
<rectangle x1="11.049" y1="-2.921" x2="11.811" y2="-1.905" layer="21"/>
<rectangle x1="13.589" y1="-2.921" x2="14.351" y2="-1.905" layer="21"/>
<rectangle x1="16.129" y1="-2.921" x2="16.891" y2="-1.905" layer="21"/>
<rectangle x1="18.669" y1="-2.921" x2="19.431" y2="-1.905" layer="21"/>
</package>
<package name="2X07" urn="urn:adsk.eagle:footprint:22370/1" locally_modified="yes" library_version="3" library_locally_modified="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-8.89" y1="-1.905" x2="-8.255" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-2.54" x2="-6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="-1.905" x2="-5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-2.54" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-2.54" x2="6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-1.905" x2="-8.89" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="1.905" x2="-8.255" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="2.54" x2="-6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="2.54" x2="-6.35" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="1.905" x2="-5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="2.54" x2="-4.445" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="4.445" y2="2.54" width="0.1524" layer="21"/>
<wire x1="4.445" y1="2.54" x2="5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="2.54" x2="6.35" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="1.905" x2="-6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="6.35" y1="1.905" x2="6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-2.54" x2="5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-2.54" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-2.54" x2="-4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.54" x2="-6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.35" y1="-1.905" x2="6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.54" x2="8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="6.35" y1="1.905" x2="6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.985" y1="2.54" x2="8.255" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.255" y1="2.54" x2="8.89" y2="1.905" width="0.1524" layer="21"/>
<wire x1="8.89" y1="1.905" x2="8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-2.54" x2="8.255" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="2" x="-7.62" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="3" x="-5.08" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="4" x="-5.08" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="5" x="-2.54" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="6" x="-2.54" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="7" x="0" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="8" x="0" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="9" x="2.54" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="10" x="2.54" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="11" x="5.08" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="12" x="5.08" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="13" x="7.62" y="-1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<pad name="14" x="7.62" y="1.27" drill="0.762" diameter="1.524" shape="octagon"/>
<text x="-8.89" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-8.89" y="-4.445" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-7.874" y1="-1.524" x2="-7.366" y2="-1.016" layer="51"/>
<rectangle x1="-7.874" y1="1.016" x2="-7.366" y2="1.524" layer="51"/>
<rectangle x1="-5.334" y1="1.016" x2="-4.826" y2="1.524" layer="51"/>
<rectangle x1="-5.334" y1="-1.524" x2="-4.826" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
<rectangle x1="4.826" y1="1.016" x2="5.334" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="4.826" y1="-1.524" x2="5.334" y2="-1.016" layer="51"/>
<rectangle x1="7.366" y1="1.016" x2="7.874" y2="1.524" layer="51"/>
<rectangle x1="7.366" y1="-1.524" x2="7.874" y2="-1.016" layer="51"/>
</package>
<package name="2X07/90" urn="urn:adsk.eagle:footprint:22371/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-8.89" y1="-1.905" x2="-6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="-1.905" x2="-6.35" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="0.635" x2="-8.89" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="0.635" x2="-8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="6.985" x2="-7.62" y2="1.27" width="0.762" layer="21"/>
<wire x1="-6.35" y1="-1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="0.635" x2="-6.35" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="6.985" x2="-5.08" y2="1.27" width="0.762" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="6.985" x2="-2.54" y2="1.27" width="0.762" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="6.985" x2="0" y2="1.27" width="0.762" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="0.635" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="6.985" x2="2.54" y2="1.27" width="0.762" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="6.35" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="6.35" y1="-1.905" x2="6.35" y2="0.635" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0.635" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="6.985" x2="5.08" y2="1.27" width="0.762" layer="21"/>
<wire x1="6.35" y1="-1.905" x2="8.89" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="8.89" y1="-1.905" x2="8.89" y2="0.635" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0.635" x2="6.35" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="6.985" x2="7.62" y2="1.27" width="0.762" layer="21"/>
<pad name="2" x="-7.62" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="4" x="-5.08" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="6" x="-2.54" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="8" x="0" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="10" x="2.54" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="12" x="5.08" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="14" x="7.62" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="1" x="-7.62" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="3" x="-5.08" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="5" x="-2.54" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="7" x="0" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="9" x="2.54" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="11" x="5.08" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="13" x="7.62" y="-6.35" drill="1.016" shape="octagon"/>
<text x="-9.525" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="10.795" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-8.001" y1="0.635" x2="-7.239" y2="1.143" layer="21"/>
<rectangle x1="-5.461" y1="0.635" x2="-4.699" y2="1.143" layer="21"/>
<rectangle x1="-2.921" y1="0.635" x2="-2.159" y2="1.143" layer="21"/>
<rectangle x1="-0.381" y1="0.635" x2="0.381" y2="1.143" layer="21"/>
<rectangle x1="2.159" y1="0.635" x2="2.921" y2="1.143" layer="21"/>
<rectangle x1="4.699" y1="0.635" x2="5.461" y2="1.143" layer="21"/>
<rectangle x1="7.239" y1="0.635" x2="8.001" y2="1.143" layer="21"/>
<rectangle x1="-8.001" y1="-2.921" x2="-7.239" y2="-1.905" layer="21"/>
<rectangle x1="-5.461" y1="-2.921" x2="-4.699" y2="-1.905" layer="21"/>
<rectangle x1="-8.001" y1="-5.461" x2="-7.239" y2="-4.699" layer="21"/>
<rectangle x1="-8.001" y1="-4.699" x2="-7.239" y2="-2.921" layer="51"/>
<rectangle x1="-5.461" y1="-4.699" x2="-4.699" y2="-2.921" layer="51"/>
<rectangle x1="-5.461" y1="-5.461" x2="-4.699" y2="-4.699" layer="21"/>
<rectangle x1="-2.921" y1="-2.921" x2="-2.159" y2="-1.905" layer="21"/>
<rectangle x1="-0.381" y1="-2.921" x2="0.381" y2="-1.905" layer="21"/>
<rectangle x1="-2.921" y1="-5.461" x2="-2.159" y2="-4.699" layer="21"/>
<rectangle x1="-2.921" y1="-4.699" x2="-2.159" y2="-2.921" layer="51"/>
<rectangle x1="-0.381" y1="-4.699" x2="0.381" y2="-2.921" layer="51"/>
<rectangle x1="-0.381" y1="-5.461" x2="0.381" y2="-4.699" layer="21"/>
<rectangle x1="2.159" y1="-2.921" x2="2.921" y2="-1.905" layer="21"/>
<rectangle x1="4.699" y1="-2.921" x2="5.461" y2="-1.905" layer="21"/>
<rectangle x1="2.159" y1="-5.461" x2="2.921" y2="-4.699" layer="21"/>
<rectangle x1="2.159" y1="-4.699" x2="2.921" y2="-2.921" layer="51"/>
<rectangle x1="4.699" y1="-4.699" x2="5.461" y2="-2.921" layer="51"/>
<rectangle x1="4.699" y1="-5.461" x2="5.461" y2="-4.699" layer="21"/>
<rectangle x1="7.239" y1="-2.921" x2="8.001" y2="-1.905" layer="21"/>
<rectangle x1="7.239" y1="-5.461" x2="8.001" y2="-4.699" layer="21"/>
<rectangle x1="7.239" y1="-4.699" x2="8.001" y2="-2.921" layer="51"/>
</package>
<package name="1X06" urn="urn:adsk.eagle:footprint:22361/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-1.27" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-7.6962" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-7.62" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
</package>
<package name="1X06/90" urn="urn:adsk.eagle:footprint:22362/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-7.62" y1="-1.905" x2="-5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="6.985" x2="-6.35" y2="1.27" width="0.762" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="6.985" x2="-3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="6.985" x2="-1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="0" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="6.985" x2="1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="6.985" x2="3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="6.35" y1="6.985" x2="6.35" y2="1.27" width="0.762" layer="21"/>
<pad name="1" x="-6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-8.255" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="9.525" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-6.731" y1="0.635" x2="-5.969" y2="1.143" layer="21"/>
<rectangle x1="-4.191" y1="0.635" x2="-3.429" y2="1.143" layer="21"/>
<rectangle x1="-1.651" y1="0.635" x2="-0.889" y2="1.143" layer="21"/>
<rectangle x1="0.889" y1="0.635" x2="1.651" y2="1.143" layer="21"/>
<rectangle x1="3.429" y1="0.635" x2="4.191" y2="1.143" layer="21"/>
<rectangle x1="5.969" y1="0.635" x2="6.731" y2="1.143" layer="21"/>
<rectangle x1="-6.731" y1="-2.921" x2="-5.969" y2="-1.905" layer="21"/>
<rectangle x1="-4.191" y1="-2.921" x2="-3.429" y2="-1.905" layer="21"/>
<rectangle x1="-1.651" y1="-2.921" x2="-0.889" y2="-1.905" layer="21"/>
<rectangle x1="0.889" y1="-2.921" x2="1.651" y2="-1.905" layer="21"/>
<rectangle x1="3.429" y1="-2.921" x2="4.191" y2="-1.905" layer="21"/>
<rectangle x1="5.969" y1="-2.921" x2="6.731" y2="-1.905" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="1X16" urn="urn:adsk.eagle:package:22432/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
<package3d name="1X16/90" urn="urn:adsk.eagle:package:22430/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
<package3d name="2X07" urn="urn:adsk.eagle:package:22478/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
<package3d name="2X07/90" urn="urn:adsk.eagle:package:22479/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
<package3d name="1X06" urn="urn:adsk.eagle:package:22472/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
<package3d name="1X06/90" urn="urn:adsk.eagle:package:22475/2" type="model" library_version="3">
<description>PIN HEADER</description>
</package3d>
</packages3d>
<symbols>
<symbol name="PINHD16" urn="urn:adsk.eagle:symbol:22296/1" library_version="3">
<wire x1="-6.35" y1="-22.86" x2="1.27" y2="-22.86" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-22.86" x2="1.27" y2="20.32" width="0.4064" layer="94"/>
<wire x1="1.27" y1="20.32" x2="-6.35" y2="20.32" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="20.32" x2="-6.35" y2="-22.86" width="0.4064" layer="94"/>
<text x="-6.35" y="20.955" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-25.4" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="17.78" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="-2.54" y="15.24" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="3" x="-2.54" y="12.7" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="4" x="-2.54" y="10.16" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="5" x="-2.54" y="7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="6" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="7" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="8" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="9" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="10" x="-2.54" y="-5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="11" x="-2.54" y="-7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="12" x="-2.54" y="-10.16" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="13" x="-2.54" y="-12.7" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="14" x="-2.54" y="-15.24" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="15" x="-2.54" y="-17.78" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="16" x="-2.54" y="-20.32" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
<symbol name="PINH2X7" urn="urn:adsk.eagle:symbol:22369/1" library_version="3">
<wire x1="-6.35" y1="-10.16" x2="8.89" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="8.89" y1="-10.16" x2="8.89" y2="10.16" width="0.4064" layer="94"/>
<wire x1="8.89" y1="10.16" x2="-6.35" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="10.16" x2="-6.35" y2="-10.16" width="0.4064" layer="94"/>
<text x="-6.35" y="10.795" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-12.7" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="5.08" y="7.62" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="3" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="4" x="5.08" y="5.08" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="5" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="6" x="5.08" y="2.54" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="7" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="8" x="5.08" y="0" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="9" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="10" x="5.08" y="-2.54" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="11" x="-2.54" y="-5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="12" x="5.08" y="-5.08" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="13" x="-2.54" y="-7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="14" x="5.08" y="-7.62" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
</symbol>
<symbol name="PINHD6" urn="urn:adsk.eagle:symbol:22360/1" library_version="3">
<wire x1="-6.35" y1="-7.62" x2="1.27" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="1.27" y2="10.16" width="0.4064" layer="94"/>
<wire x1="1.27" y1="10.16" x2="-6.35" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="10.16" x2="-6.35" y2="-7.62" width="0.4064" layer="94"/>
<text x="-6.35" y="10.795" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="3" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="4" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="5" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="6" x="-2.54" y="-5.08" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="PINHD-1X16" urn="urn:adsk.eagle:component:22522/3" prefix="JP" uservalue="yes" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINHD16" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1X16">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="11" pad="11"/>
<connect gate="A" pin="12" pad="12"/>
<connect gate="A" pin="13" pad="13"/>
<connect gate="A" pin="14" pad="14"/>
<connect gate="A" pin="15" pad="15"/>
<connect gate="A" pin="16" pad="16"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22432/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="1X16/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="11" pad="11"/>
<connect gate="A" pin="12" pad="12"/>
<connect gate="A" pin="13" pad="13"/>
<connect gate="A" pin="14" pad="14"/>
<connect gate="A" pin="15" pad="15"/>
<connect gate="A" pin="16" pad="16"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22430/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PINHD-2X7" urn="urn:adsk.eagle:component:22536/3" locally_modified="yes" prefix="JP" uservalue="yes" library_version="3" library_locally_modified="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINH2X7" x="0" y="0"/>
</gates>
<devices>
<device name="" package="2X07">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="11" pad="11"/>
<connect gate="A" pin="12" pad="12"/>
<connect gate="A" pin="13" pad="13"/>
<connect gate="A" pin="14" pad="14"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22478/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="2X07/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="11" pad="11"/>
<connect gate="A" pin="12" pad="12"/>
<connect gate="A" pin="13" pad="13"/>
<connect gate="A" pin="14" pad="14"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22479/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PINHD-1X6" urn="urn:adsk.eagle:component:22533/3" prefix="JP" uservalue="yes" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINHD6" x="0" y="-2.54"/>
</gates>
<devices>
<device name="" package="1X06">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22472/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="1X06/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22475/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply2" urn="urn:adsk.eagle:library:372">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
Please keep in mind, that these devices are necessary for the
automatic wiring of the supply signals.&lt;p&gt;
The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26990/1" library_version="2">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:27037/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="to-247_n-channel_igbt">
<packages>
<package name="TO-247">
<wire x1="2.5" y1="-8" x2="2.5" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2.5" y1="-1.5" x2="2.5" y2="1.5" width="0.127" layer="21"/>
<wire x1="2.5" y1="1.5" x2="2.5" y2="8" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-8" x2="-2.5" y2="-7.5" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-7.5" x2="-2.5" y2="-1.5" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-1.5" x2="-2.5" y2="1.5" width="0.127" layer="21"/>
<wire x1="-2.5" y1="1.5" x2="-2.5" y2="7.5" width="0.127" layer="21"/>
<wire x1="-2.5" y1="7.5" x2="-2.5" y2="8" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-8" x2="-1.5" y2="-8" width="0.127" layer="21"/>
<wire x1="-1.5" y1="-8" x2="2.5" y2="-8" width="0.127" layer="21"/>
<wire x1="-2.5" y1="8" x2="-1" y2="8" width="0.127" layer="21"/>
<wire x1="-1" y1="8" x2="2.5" y2="8" width="0.127" layer="21"/>
<wire x1="-2.5" y1="7.5" x2="-1" y2="7.5" width="0.127" layer="21"/>
<wire x1="-1" y1="7.5" x2="-1" y2="8" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-7.5" x2="-1.5" y2="-7.5" width="0.127" layer="21"/>
<wire x1="-1.5" y1="-7.5" x2="-1.5" y2="-8" width="0.127" layer="21"/>
<wire x1="2.5" y1="-8" x2="3" y2="-8" width="0.127" layer="21"/>
<wire x1="3" y1="-8" x2="3" y2="8" width="0.127" layer="21"/>
<wire x1="3" y1="8" x2="2.5" y2="8" width="0.127" layer="21"/>
<wire x1="2.5" y1="1.5" x2="-2.5" y2="1.5" width="0.127" layer="21"/>
<wire x1="0" y1="-1.5" x2="-2.5" y2="-1.5" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-1.5" x2="2.5" y2="-1.5" width="0.127" layer="21"/>
<wire x1="3" y1="8" x2="4" y2="8" width="0.127" layer="21"/>
<wire x1="4" y1="8" x2="4" y2="-8" width="0.127" layer="21"/>
<wire x1="4" y1="-8" x2="3" y2="-8" width="0.127" layer="21"/>
<pad name="P$1" x="0" y="5.5" drill="0.8" diameter="3.81" shape="long"/>
<pad name="P$2" x="0" y="0" drill="0.8" diameter="3.81" shape="long"/>
<pad name="P$3" x="0" y="-5.5" drill="0.8" diameter="3.81" shape="long"/>
<text x="-5.08" y="7.62" size="1.27" layer="21">G</text>
<text x="-5.08" y="1.27" size="1.27" layer="21">C</text>
<text x="-5.08" y="-3.81" size="1.27" layer="21">E</text>
<text x="-2.54" y="8.89" size="1.27" layer="21">1</text>
<rectangle x1="2.5" y1="-8" x2="3" y2="8" layer="21"/>
<rectangle x1="2.5" y1="-8" x2="4" y2="8" layer="21"/>
</package>
<package name="TO-247-FLAT">
<wire x1="-0.5" y1="0.5" x2="0.5" y2="0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="0.5" x2="0.5" y2="-0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-0.5" x2="-0.5" y2="-0.5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-0.5" x2="-0.5" y2="0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-5" x2="-0.5" y2="-5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-5" x2="-0.5" y2="-6" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-6" x2="0.5" y2="-6" width="0.127" layer="20"/>
<wire x1="0.5" y1="-6" x2="0.5" y2="-5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="6" x2="0.5" y2="6" width="0.127" layer="20"/>
<wire x1="0.5" y1="6" x2="0.5" y2="5" width="0.127" layer="20"/>
<wire x1="0.5" y1="5" x2="-0.5" y2="5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="5" x2="-0.5" y2="6" width="0.127" layer="20"/>
<wire x1="7" y1="-8" x2="7" y2="-6" width="0.127" layer="20"/>
<wire x1="7" y1="-6" x2="7" y2="-4" width="0.127" layer="20"/>
<wire x1="7" y1="-4" x2="7" y2="-1.5" width="0.127" layer="20"/>
<wire x1="7" y1="-1.5" x2="7" y2="1.5" width="0.127" layer="20"/>
<wire x1="7" y1="1.5" x2="7" y2="4" width="0.127" layer="20"/>
<wire x1="7" y1="6" x2="7" y2="8" width="0.127" layer="20"/>
<wire x1="7" y1="-8" x2="19" y2="-8" width="0.127" layer="20"/>
<wire x1="19" y1="-8" x2="19.5" y2="-8" width="0.127" layer="20"/>
<wire x1="19.5" y1="-8" x2="23.5" y2="-8" width="0.127" layer="20"/>
<wire x1="23.5" y1="-8" x2="24" y2="-8" width="0.127" layer="20"/>
<wire x1="24" y1="-8" x2="27" y2="-8" width="0.127" layer="20"/>
<wire x1="27" y1="-8" x2="27" y2="8" width="0.127" layer="20"/>
<wire x1="27" y1="8" x2="24" y2="8" width="0.127" layer="20"/>
<wire x1="24" y1="8" x2="23.5" y2="8" width="0.127" layer="20"/>
<wire x1="23.5" y1="8" x2="19.5" y2="8" width="0.127" layer="20"/>
<wire x1="19.5" y1="8" x2="19" y2="8" width="0.127" layer="20"/>
<wire x1="19" y1="8" x2="7" y2="8" width="0.127" layer="20"/>
<wire x1="19" y1="0" x2="21.5" y2="0" width="0.127" layer="20"/>
<wire x1="25" y1="0" x2="24" y2="0" width="0.127" layer="20"/>
<wire x1="21.5" y1="0" x2="21.5" y2="0.5" width="0.127" layer="20"/>
<wire x1="19.5" y1="-8" x2="23.5" y2="-8" width="0.127" layer="20" curve="-180"/>
<wire x1="23.5" y1="8" x2="19.5" y2="8" width="0.127" layer="20" curve="-180"/>
<wire x1="24" y1="8" x2="19" y2="8" width="0.127" layer="20" curve="-180"/>
<wire x1="19" y1="-8" x2="24" y2="-8" width="0.127" layer="20" curve="-180"/>
<wire x1="24" y1="0" x2="19" y2="0" width="0.127" layer="20" curve="-180"/>
<wire x1="19" y1="0" x2="24" y2="0" width="0.127" layer="20" curve="-180"/>
<wire x1="9" y1="5.5" x2="16.5" y2="5.5" width="0.127" layer="20"/>
<wire x1="16.5" y1="5.5" x2="16.5" y2="-5.5" width="0.127" layer="20"/>
<wire x1="16.5" y1="-5.5" x2="9" y2="-5.5" width="0.127" layer="20"/>
<wire x1="9" y1="-5.5" x2="9" y2="5.5" width="0.127" layer="20"/>
<wire x1="7" y1="1.5" x2="6.5" y2="1.5" width="0.127" layer="20"/>
<wire x1="6.5" y1="-1.5" x2="7" y2="-1.5" width="0.127" layer="20"/>
<wire x1="7" y1="4" x2="7" y2="6" width="0.127" layer="20"/>
<wire x1="3" y1="5" x2="3" y2="4" width="0.127" layer="20"/>
<wire x1="3" y1="4" x2="7" y2="4" width="0.127" layer="20"/>
<wire x1="7" y1="1.5" x2="3" y2="1.5" width="0.127" layer="20"/>
<wire x1="3" y1="1.5" x2="3" y2="0.5" width="0.127" layer="20"/>
<wire x1="3" y1="-0.5" x2="3" y2="-1.5" width="0.127" layer="20"/>
<wire x1="3" y1="-1.5" x2="7" y2="-1.5" width="0.127" layer="20"/>
<wire x1="7" y1="-4" x2="3" y2="-4" width="0.127" layer="20"/>
<wire x1="3" y1="-4" x2="3" y2="-5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-5" x2="3" y2="-5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-6" x2="7" y2="-6" width="0.127" layer="20"/>
<wire x1="0.5" y1="-0.5" x2="3" y2="-0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="0.5" x2="3" y2="0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="5" x2="3" y2="5" width="0.127" layer="20"/>
<wire x1="7" y1="6" x2="0.5" y2="6" width="0.127" layer="20"/>
<wire x1="18" y1="0" x2="21.5" y2="0" width="0.127" layer="20"/>
<wire x1="24" y1="0" x2="21.5" y2="0" width="0.127" layer="20"/>
<wire x1="21.5" y1="0" x2="21.5" y2="3.5" width="0.127" layer="20"/>
<wire x1="21.5" y1="0" x2="21.5" y2="-3.5" width="0.127" layer="20"/>
<circle x="21.5" y="0" radius="2" width="0.127" layer="20"/>
<pad name="P$1" x="0" y="5.5" drill="0.8" diameter="3.81" shape="long"/>
<pad name="P$2" x="0" y="0" drill="0.8" diameter="3.81" shape="long"/>
<pad name="P$3" x="0" y="-5.5" drill="0.8" diameter="3.81" shape="long"/>
<text x="-5.08" y="7.62" size="1.27" layer="21">G</text>
<text x="-5.08" y="1.27" size="1.27" layer="21">C</text>
<text x="-5.08" y="-3.81" size="1.27" layer="21">E</text>
<text x="-2.54" y="7.62" size="1.27" layer="21">1</text>
</package>
<package name="TO-247-FLATUD">
<wire x1="-0.5" y1="0.5" x2="0.5" y2="0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="0.5" x2="0.5" y2="-0.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-0.5" x2="-0.5" y2="-0.5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-0.5" x2="-0.5" y2="0.5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="6" x2="0.5" y2="6" width="0.127" layer="20"/>
<wire x1="0.5" y1="6" x2="0.5" y2="5" width="0.127" layer="20"/>
<wire x1="0.5" y1="5" x2="-0.5" y2="5" width="0.127" layer="20"/>
<wire x1="-0.5" y1="5" x2="-0.5" y2="6" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-5" x2="0.5" y2="-5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-5" x2="0.5" y2="-6" width="0.127" layer="20"/>
<wire x1="0.5" y1="-6" x2="-0.5" y2="-6" width="0.127" layer="20"/>
<wire x1="-0.5" y1="-6" x2="-0.5" y2="-5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-6" x2="7" y2="-6" width="0.127" layer="20"/>
<wire x1="7" y1="-6" x2="7" y2="-4" width="0.127" layer="20"/>
<wire x1="7" y1="-4" x2="7" y2="-1.5" width="0.127" layer="20"/>
<wire x1="7" y1="-1.5" x2="7" y2="1.5" width="0.127" layer="20"/>
<wire x1="7" y1="1.5" x2="7" y2="4" width="0.127" layer="20"/>
<wire x1="7" y1="4" x2="7" y2="6" width="0.127" layer="20"/>
<wire x1="7" y1="6" x2="0.5" y2="6" width="0.127" layer="20"/>
<wire x1="0.5" y1="-5" x2="3" y2="-5" width="0.127" layer="20"/>
<wire x1="3" y1="-5" x2="3" y2="-4" width="0.127" layer="20"/>
<wire x1="3" y1="-4" x2="7" y2="-4" width="0.127" layer="20"/>
<wire x1="1" y1="5" x2="0.5" y2="5" width="0.127" layer="20"/>
<wire x1="0.5" y1="5" x2="3" y2="5" width="0.127" layer="20"/>
<wire x1="3" y1="5" x2="3" y2="4" width="0.127" layer="20"/>
<wire x1="3" y1="4" x2="7" y2="4" width="0.127" layer="20"/>
<wire x1="0.5" y1="0.5" x2="3" y2="0.5" width="0.127" layer="20"/>
<wire x1="3" y1="0.5" x2="3" y2="1.5" width="0.127" layer="20"/>
<wire x1="3" y1="1.5" x2="7" y2="1.5" width="0.127" layer="20"/>
<wire x1="0.5" y1="-0.5" x2="3" y2="-0.5" width="0.127" layer="20"/>
<wire x1="3" y1="-0.5" x2="3" y2="-1.5" width="0.127" layer="20"/>
<wire x1="3" y1="-1.5" x2="7" y2="-1.5" width="0.127" layer="20"/>
<wire x1="7" y1="-6" x2="7" y2="-8" width="0.127" layer="20"/>
<wire x1="7" y1="6" x2="7" y2="8" width="0.127" layer="20"/>
<wire x1="7" y1="-8" x2="19.5" y2="-8" width="0.127" layer="20"/>
<wire x1="19.5" y1="-8" x2="23.5" y2="-8" width="0.127" layer="20"/>
<wire x1="23.5" y1="-8" x2="27" y2="-8" width="0.127" layer="20"/>
<wire x1="27" y1="-8" x2="27" y2="8" width="0.127" layer="20"/>
<wire x1="27" y1="8" x2="23.5" y2="8" width="0.127" layer="20"/>
<wire x1="23.5" y1="8" x2="19.5" y2="8" width="0.127" layer="20"/>
<wire x1="19.5" y1="8" x2="7" y2="8" width="0.127" layer="20"/>
<wire x1="8.5" y1="6.5" x2="25.5" y2="6.5" width="0.127" layer="20"/>
<wire x1="25.5" y1="6.5" x2="25.5" y2="-6.5" width="0.127" layer="20"/>
<wire x1="25.5" y1="-6.5" x2="8.5" y2="-6.5" width="0.127" layer="20"/>
<wire x1="8.5" y1="-6.5" x2="8.5" y2="6.5" width="0.127" layer="20"/>
<wire x1="21.5" y1="3.5" x2="21.5" y2="-3.5" width="0.127" layer="20"/>
<wire x1="18" y1="0" x2="25" y2="0" width="0.127" layer="20"/>
<wire x1="19.5" y1="8" x2="19.5" y2="7.5" width="0.127" layer="20"/>
<wire x1="19.5" y1="7.5" x2="23.5" y2="7.5" width="0.127" layer="20"/>
<wire x1="23.5" y1="7.5" x2="23.5" y2="8" width="0.127" layer="20"/>
<wire x1="19.5" y1="-8" x2="19.5" y2="-7.5" width="0.127" layer="20"/>
<wire x1="19.5" y1="-7.5" x2="23.5" y2="-7.5" width="0.127" layer="20"/>
<wire x1="23.5" y1="-7.5" x2="23.5" y2="-8" width="0.127" layer="20"/>
<circle x="21.5" y="0" radius="2" width="0.127" layer="20"/>
<circle x="21.5" y="0" radius="2.5" width="0.127" layer="20"/>
<pad name="P$1" x="0" y="-5.5" drill="1" diameter="3.81" shape="long"/>
<pad name="P$2" x="0" y="0" drill="1" diameter="3.81" shape="long"/>
<pad name="P$3" x="0" y="5.5" drill="1" diameter="3.81" shape="long"/>
<text x="-2" y="-9.5" size="1.27" layer="25">1</text>
<text x="-5.5" y="-4.5" size="1.27" layer="25">G</text>
<text x="-5.5" y="1.5" size="1.27" layer="25">C</text>
<text x="-5.5" y="7" size="1.27" layer="25">E</text>
</package>
</packages>
<symbols>
<symbol name="TO-247">
<wire x1="-1.5" y1="-2" x2="-1.5" y2="-1.5" width="0.254" layer="94"/>
<wire x1="-1.5" y1="-1.5" x2="-1.5" y2="-1" width="0.254" layer="94"/>
<wire x1="-1.5" y1="-0.5" x2="-1.5" y2="0" width="0.254" layer="94"/>
<wire x1="-1.5" y1="0" x2="-1.5" y2="0.5" width="0.254" layer="94"/>
<wire x1="-1.5" y1="1" x2="-1.5" y2="1.5" width="0.254" layer="94"/>
<wire x1="-1.5" y1="1.5" x2="-1.5" y2="2" width="0.254" layer="94"/>
<wire x1="-2.25" y1="2" x2="-2.25" y2="-1.25" width="0.254" layer="94"/>
<wire x1="-2.25" y1="-1.25" x2="-2.25" y2="-2" width="0.254" layer="94"/>
<wire x1="-1.5" y1="-1.5" x2="-0.5" y2="-2.5" width="0.254" layer="94"/>
<wire x1="-0.5" y1="-2.5" x2="0" y2="-3" width="0.254" layer="94"/>
<wire x1="-1.5" y1="1.5" x2="-1" y2="2" width="0.254" layer="94"/>
<wire x1="-1" y1="2" x2="0" y2="3" width="0.254" layer="94"/>
<wire x1="-1" y1="2.5" x2="-1" y2="2" width="0.254" layer="94"/>
<wire x1="-1" y1="2" x2="-0.5" y2="2" width="0.254" layer="94"/>
<wire x1="-0.5" y1="2" x2="-1" y2="2.5" width="0.254" layer="94"/>
<wire x1="-0.5" y1="-2.5" x2="-0.5" y2="-2" width="0.254" layer="94"/>
<wire x1="-0.5" y1="-2" x2="-1" y2="-2.5" width="0.254" layer="94"/>
<wire x1="-1" y1="-2.5" x2="-0.5" y2="-2.5" width="0.254" layer="94"/>
<wire x1="0.5" y1="0.5" x2="1.5" y2="0.5" width="0.254" layer="94"/>
<wire x1="2.5" y1="0.5" x2="1.5" y2="0.5" width="0.254" layer="94"/>
<wire x1="1.5" y1="0.5" x2="0.5" y2="-0.5" width="0.254" layer="94"/>
<wire x1="0.5" y1="-0.5" x2="1.5" y2="-0.5" width="0.254" layer="94"/>
<wire x1="1.5" y1="-0.5" x2="2.5" y2="-0.5" width="0.254" layer="94"/>
<wire x1="2.5" y1="-0.5" x2="1.5" y2="0.5" width="0.254" layer="94"/>
<wire x1="0" y1="3" x2="1.5" y2="3" width="0.254" layer="94"/>
<wire x1="1.5" y1="3" x2="1.5" y2="0.5" width="0.254" layer="94"/>
<wire x1="0" y1="-3" x2="1.5" y2="-3" width="0.254" layer="94"/>
<wire x1="1.5" y1="-3" x2="1.5" y2="-0.5" width="0.254" layer="94"/>
<wire x1="-1.5" y1="0" x2="0" y2="-1.5" width="0.254" layer="94"/>
<wire x1="-3.5" y1="-2.5" x2="-2.25" y2="-1.25" width="0.254" layer="94"/>
<wire x1="-3.5" y1="-2.5" x2="-10.25" y2="-2.5" width="0.254" layer="94"/>
<wire x1="0" y1="-3" x2="0" y2="-10.25" width="0.254" layer="94"/>
<wire x1="0" y1="3" x2="0" y2="10.25" width="0.254" layer="94"/>
<circle x="0" y="0" radius="4" width="0.254" layer="94"/>
<text x="-7" y="-1" size="1.778" layer="94">G</text>
<text x="1" y="5.5" size="1.778" layer="94">C</text>
<text x="1.5" y="-7" size="1.778" layer="94">E</text>
<text x="-7.5" y="5" size="1.27" layer="94">N-ch</text>
<pin name="G" x="-10.16" y="-2.54" length="middle"/>
<pin name="C" x="0" y="10.16" length="middle" rot="R270"/>
<pin name="E" x="0" y="-10.16" length="middle" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TO-247">
<gates>
<gate name="G$1" symbol="TO-247" x="0" y="0"/>
</gates>
<devices>
<device name="A" package="TO-247">
<connects>
<connect gate="G$1" pin="C" pad="P$2"/>
<connect gate="G$1" pin="E" pad="P$3"/>
<connect gate="G$1" pin="G" pad="P$1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B" package="TO-247-FLAT">
<connects>
<connect gate="G$1" pin="C" pad="P$2"/>
<connect gate="G$1" pin="E" pad="P$3"/>
<connect gate="G$1" pin="G" pad="P$1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C" package="TO-247-FLATUD">
<connects>
<connect gate="G$1" pin="C" pad="P$2"/>
<connect gate="G$1" pin="E" pad="P$3"/>
<connect gate="G$1" pin="G" pad="P$1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="docu-dummy" urn="urn:adsk.eagle:library:215">
<description>Dummy symbols</description>
<packages>
</packages>
<symbols>
<symbol name="CAPACITOR" urn="urn:adsk.eagle:symbol:13164/1" library_version="1">
<wire x1="0" y1="-5.08" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="-0.508" x2="0" y2="2.54" width="0.1524" layer="94"/>
<rectangle x1="-2.032" y1="-2.032" x2="2.032" y2="-1.524" layer="94"/>
<rectangle x1="-2.032" y1="-1.016" x2="2.032" y2="-0.508" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C" urn="urn:adsk.eagle:component:13175/1" prefix="C" library_version="1">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IGB1.2_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="1.2"/>
<part name="IGB1.1_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="1.1"/>
<part name="FONTE1" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2"/>
<part name="JP1" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-2X7" device="" package3d_urn="urn:adsk.eagle:package:22478/2"/>
<part name="SUPPLY1" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY2" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY4" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="IGB2.2_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="2.2"/>
<part name="IGB2.1_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="2.1"/>
<part name="JP2" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-2X7" device="" package3d_urn="urn:adsk.eagle:package:22478/2"/>
<part name="SUPPLY5" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY6" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="FONTE2.2" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X6" device="" package3d_urn="urn:adsk.eagle:package:22472/2"/>
<part name="FONTE2" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X6" device="" package3d_urn="urn:adsk.eagle:package:22472/2"/>
<part name="IGB3.2_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="3.2"/>
<part name="IGB3.1_DRIVER" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X16" device="" package3d_urn="urn:adsk.eagle:package:22432/2" value="3.1"/>
<part name="JP3" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-2X7" device="" package3d_urn="urn:adsk.eagle:package:22478/2"/>
<part name="SUPPLY3" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY7" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="FONTE3.2" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X6" device="" package3d_urn="urn:adsk.eagle:package:22472/2"/>
<part name="FONTE3" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X6" device="" package3d_urn="urn:adsk.eagle:package:22472/2"/>
<part name="U$1" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="U$2" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="U$3" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="U$4" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="U$5" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="U$6" library="to-247_n-channel_igbt" deviceset="TO-247" device="A"/>
<part name="C1" library="docu-dummy" library_urn="urn:adsk.eagle:library:215" deviceset="C" device=""/>
<part name="C2" library="docu-dummy" library_urn="urn:adsk.eagle:library:215" deviceset="C" device=""/>
<part name="C3" library="docu-dummy" library_urn="urn:adsk.eagle:library:215" deviceset="C" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<text x="76.2" y="115.57" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">CSa



GSa
ESa
T11
T12


RST
Sa
ER
VS
GND
</text>
<text x="76.2" y="49.53" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">



Sa



T22
T21
ESb
GSb



CSb</text>
<text x="52.07" y="115.57" size="1.4224" layer="97" ratio="30" distance="80" rot="R180" align="center-right">T11
T12



T21
T22




Tp1
Tp2
VS
GND
</text>
<circle x="123.19" y="83.82" radius="2.54" width="0.2" layer="91"/>
<text x="123.19" y="81.28" size="2.54" layer="97" rot="R270">VA</text>
<circle x="6.35" y="27.94" radius="2.54" width="0.2" layer="91"/>
<text x="6.35" y="25.4" size="2.54" layer="97" rot="R270">DC-</text>
<circle x="5.08" y="80.01" radius="2.54" width="0.2" layer="91"/>
<text x="5.08" y="77.47" size="2.54" layer="97" rot="R270">Neutral</text>
<circle x="5.08" y="130.81" radius="2.54" width="0.2" layer="91"/>
<text x="5.08" y="128.27" size="2.54" layer="97" rot="R270">DC+</text>
<text x="187.96" y="115.57" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">CSa



GSa
ESa
T11
T12


RST
Sa
ER
VS
GND
</text>
<text x="187.96" y="49.53" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">



Sa



T22
T21
ESb
GSb



CSb</text>
<circle x="234.95" y="83.82" radius="2.54" width="0.2" layer="91"/>
<text x="234.95" y="81.28" size="2.54" layer="97" rot="R270">VB</text>
<text x="149.86" y="114.3" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">Tp2


T22

T21</text>
<text x="171.45" y="114.3" size="1.4224" layer="97" ratio="30" distance="80" rot="R180" align="center-right">Tp1


T12

T11</text>
<text x="307.34" y="115.57" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">CSa



GSa
ESa
T11
T12


RST
Sa
ER
VS
GND
</text>
<text x="307.34" y="49.53" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">



Sa



T22
T21
ESb
GSb



CSb</text>
<circle x="354.33" y="83.82" radius="2.54" width="0.2" layer="91"/>
<text x="354.33" y="81.28" size="2.54" layer="97" rot="R270">VC</text>
<text x="269.24" y="114.3" size="1.4224" layer="97" ratio="30" distance="80" align="center-right">Tp2


T22

T21</text>
<text x="290.83" y="114.3" size="1.4224" layer="97" ratio="30" distance="80" rot="R180" align="center-right">Tp1


T12

T11</text>
</plain>
<instances>
<instance part="IGB1.2_DRIVER" gate="A" x="78.74" y="48.26" rot="R180">
<attribute name="NAME" x="85.09" y="27.305" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="85.09" y="73.66" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IGB1.1_DRIVER" gate="A" x="78.74" y="114.3" rot="R180">
<attribute name="NAME" x="85.09" y="93.345" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="85.09" y="139.7" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="FONTE1" gate="A" x="49.53" y="116.84">
<attribute name="NAME" x="43.18" y="137.795" size="1.778" layer="95"/>
<attribute name="VALUE" x="43.18" y="91.44" size="1.778" layer="96"/>
</instance>
<instance part="JP1" gate="A" x="46.99" y="44.45">
<attribute name="NAME" x="40.64" y="55.245" size="1.778" layer="95"/>
<attribute name="VALUE" x="40.64" y="31.75" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY1" gate="GND" x="39.37" y="96.52">
<attribute name="VALUE" x="37.465" y="93.345" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY2" gate="GND" x="82.55" y="87.63">
<attribute name="VALUE" x="80.645" y="84.455" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY4" gate="GND" x="34.29" y="44.45">
<attribute name="VALUE" x="32.385" y="41.275" size="1.778" layer="96"/>
</instance>
<instance part="IGB2.2_DRIVER" gate="A" x="190.5" y="48.26" rot="R180">
<attribute name="NAME" x="196.85" y="27.305" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="196.85" y="73.66" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IGB2.1_DRIVER" gate="A" x="190.5" y="114.3" rot="R180">
<attribute name="NAME" x="196.85" y="93.345" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="196.85" y="139.7" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="JP2" gate="A" x="158.75" y="44.45">
<attribute name="NAME" x="152.4" y="55.245" size="1.778" layer="95"/>
<attribute name="VALUE" x="152.4" y="31.75" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY5" gate="GND" x="194.31" y="87.63">
<attribute name="VALUE" x="192.405" y="84.455" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY6" gate="GND" x="146.05" y="44.45">
<attribute name="VALUE" x="144.145" y="41.275" size="1.778" layer="96"/>
</instance>
<instance part="FONTE2.2" gate="A" x="161.29" y="115.57" rot="R180">
<attribute name="VALUE" x="167.64" y="125.73" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="FONTE2" gate="A" x="158.75" y="113.03">
<attribute name="NAME" x="152.4" y="123.825" size="1.778" layer="95"/>
<attribute name="VALUE" x="152.4" y="102.87" size="1.778" layer="96"/>
</instance>
<instance part="IGB3.2_DRIVER" gate="A" x="309.88" y="48.26" rot="R180">
<attribute name="NAME" x="316.23" y="27.305" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="316.23" y="73.66" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IGB3.1_DRIVER" gate="A" x="309.88" y="114.3" rot="R180">
<attribute name="NAME" x="316.23" y="93.345" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="316.23" y="139.7" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="JP3" gate="A" x="278.13" y="44.45">
<attribute name="NAME" x="271.78" y="55.245" size="1.778" layer="95"/>
<attribute name="VALUE" x="271.78" y="31.75" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY3" gate="GND" x="313.69" y="87.63">
<attribute name="VALUE" x="311.785" y="84.455" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY7" gate="GND" x="265.43" y="44.45">
<attribute name="VALUE" x="263.525" y="41.275" size="1.778" layer="96"/>
</instance>
<instance part="FONTE3.2" gate="A" x="280.67" y="115.57" rot="R180">
<attribute name="VALUE" x="287.02" y="125.73" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="FONTE3" gate="A" x="278.13" y="113.03">
<attribute name="NAME" x="271.78" y="123.825" size="1.778" layer="95"/>
<attribute name="VALUE" x="271.78" y="102.87" size="1.778" layer="96"/>
</instance>
<instance part="U$1" gate="G$1" x="110.49" y="129.54"/>
<instance part="U$2" gate="G$1" x="110.49" y="43.18"/>
<instance part="U$3" gate="G$1" x="222.25" y="129.54"/>
<instance part="U$4" gate="G$1" x="222.25" y="43.18"/>
<instance part="U$5" gate="G$1" x="341.63" y="129.54"/>
<instance part="U$6" gate="G$1" x="341.63" y="43.18"/>
<instance part="C1" gate="G$1" x="132.08" y="85.09"/>
<instance part="C2" gate="G$1" x="246.38" y="85.09"/>
<instance part="C3" gate="G$1" x="365.76" y="85.09"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="FONTE1" gate="A" pin="15"/>
<pinref part="SUPPLY1" gate="GND" pin="GND"/>
<wire x1="46.99" y1="99.06" x2="39.37" y2="99.06" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="2"/>
<pinref part="SUPPLY2" gate="GND" pin="GND"/>
<wire x1="81.28" y1="99.06" x2="87.63" y2="99.06" width="0.1524" layer="91"/>
<wire x1="87.63" y1="99.06" x2="87.63" y2="90.17" width="0.1524" layer="91"/>
<wire x1="87.63" y1="90.17" x2="82.55" y2="90.17" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="JP1" gate="A" pin="1"/>
<wire x1="41.91" y1="52.07" x2="44.45" y2="52.07" width="0.1524" layer="91"/>
<pinref part="JP1" gate="A" pin="5"/>
<pinref part="JP1" gate="A" pin="11"/>
<wire x1="34.29" y1="46.99" x2="38.1" y2="46.99" width="0.1524" layer="91"/>
<pinref part="SUPPLY4" gate="GND" pin="GND"/>
<pinref part="JP1" gate="A" pin="10"/>
<wire x1="38.1" y1="46.99" x2="44.45" y2="46.99" width="0.1524" layer="91"/>
<wire x1="52.07" y1="41.91" x2="57.15" y2="41.91" width="0.1524" layer="91"/>
<wire x1="57.15" y1="41.91" x2="57.15" y2="33.02" width="0.1524" layer="91"/>
<wire x1="57.15" y1="33.02" x2="38.1" y2="33.02" width="0.1524" layer="91"/>
<wire x1="38.1" y1="33.02" x2="38.1" y2="39.37" width="0.1524" layer="91"/>
<wire x1="44.45" y1="39.37" x2="38.1" y2="39.37" width="0.1524" layer="91"/>
<wire x1="44.45" y1="52.07" x2="38.1" y2="52.07" width="0.1524" layer="91"/>
<junction x="44.45" y="52.07"/>
<wire x1="38.1" y1="52.07" x2="38.1" y2="46.99" width="0.1524" layer="91"/>
<junction x="38.1" y="46.99"/>
<wire x1="38.1" y1="46.99" x2="38.1" y2="39.37" width="0.1524" layer="91"/>
<junction x="38.1" y="39.37"/>
</segment>
<segment>
<pinref part="IGB2.1_DRIVER" gate="A" pin="2"/>
<pinref part="SUPPLY5" gate="GND" pin="GND"/>
<wire x1="193.04" y1="99.06" x2="199.39" y2="99.06" width="0.1524" layer="91"/>
<wire x1="199.39" y1="99.06" x2="199.39" y2="90.17" width="0.1524" layer="91"/>
<wire x1="199.39" y1="90.17" x2="194.31" y2="90.17" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="JP2" gate="A" pin="1"/>
<wire x1="153.67" y1="52.07" x2="156.21" y2="52.07" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="5"/>
<pinref part="JP2" gate="A" pin="11"/>
<wire x1="146.05" y1="46.99" x2="149.86" y2="46.99" width="0.1524" layer="91"/>
<pinref part="SUPPLY6" gate="GND" pin="GND"/>
<pinref part="JP2" gate="A" pin="10"/>
<wire x1="149.86" y1="46.99" x2="156.21" y2="46.99" width="0.1524" layer="91"/>
<wire x1="163.83" y1="41.91" x2="168.91" y2="41.91" width="0.1524" layer="91"/>
<wire x1="168.91" y1="41.91" x2="168.91" y2="33.02" width="0.1524" layer="91"/>
<wire x1="168.91" y1="33.02" x2="149.86" y2="33.02" width="0.1524" layer="91"/>
<wire x1="149.86" y1="33.02" x2="149.86" y2="39.37" width="0.1524" layer="91"/>
<wire x1="156.21" y1="39.37" x2="149.86" y2="39.37" width="0.1524" layer="91"/>
<wire x1="156.21" y1="52.07" x2="149.86" y2="52.07" width="0.1524" layer="91"/>
<junction x="156.21" y="52.07"/>
<wire x1="149.86" y1="52.07" x2="149.86" y2="46.99" width="0.1524" layer="91"/>
<junction x="149.86" y="46.99"/>
<wire x1="149.86" y1="46.99" x2="149.86" y2="39.37" width="0.1524" layer="91"/>
<junction x="149.86" y="39.37"/>
</segment>
<segment>
<pinref part="IGB3.1_DRIVER" gate="A" pin="2"/>
<pinref part="SUPPLY3" gate="GND" pin="GND"/>
<wire x1="312.42" y1="99.06" x2="318.77" y2="99.06" width="0.1524" layer="91"/>
<wire x1="318.77" y1="99.06" x2="318.77" y2="90.17" width="0.1524" layer="91"/>
<wire x1="318.77" y1="90.17" x2="313.69" y2="90.17" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="JP3" gate="A" pin="1"/>
<wire x1="273.05" y1="52.07" x2="275.59" y2="52.07" width="0.1524" layer="91"/>
<pinref part="JP3" gate="A" pin="5"/>
<pinref part="JP3" gate="A" pin="11"/>
<wire x1="265.43" y1="46.99" x2="269.24" y2="46.99" width="0.1524" layer="91"/>
<pinref part="SUPPLY7" gate="GND" pin="GND"/>
<pinref part="JP3" gate="A" pin="10"/>
<wire x1="269.24" y1="46.99" x2="275.59" y2="46.99" width="0.1524" layer="91"/>
<wire x1="283.21" y1="41.91" x2="288.29" y2="41.91" width="0.1524" layer="91"/>
<wire x1="288.29" y1="41.91" x2="288.29" y2="33.02" width="0.1524" layer="91"/>
<wire x1="288.29" y1="33.02" x2="269.24" y2="33.02" width="0.1524" layer="91"/>
<wire x1="269.24" y1="33.02" x2="269.24" y2="39.37" width="0.1524" layer="91"/>
<wire x1="275.59" y1="39.37" x2="269.24" y2="39.37" width="0.1524" layer="91"/>
<wire x1="275.59" y1="52.07" x2="269.24" y2="52.07" width="0.1524" layer="91"/>
<junction x="275.59" y="52.07"/>
<wire x1="269.24" y1="52.07" x2="269.24" y2="46.99" width="0.1524" layer="91"/>
<junction x="269.24" y="46.99"/>
<wire x1="269.24" y1="46.99" x2="269.24" y2="39.37" width="0.1524" layer="91"/>
<junction x="269.24" y="39.37"/>
</segment>
</net>
<net name="VS" class="0">
<segment>
<wire x1="91.44" y1="101.6" x2="91.44" y2="81.28" width="0.1524" layer="91"/>
<pinref part="IGB1.1_DRIVER" gate="A" pin="3"/>
<wire x1="81.28" y1="101.6" x2="91.44" y2="101.6" width="0.1524" layer="91"/>
<wire x1="91.44" y1="81.28" x2="52.07" y2="81.28" width="0.1524" layer="91"/>
<wire x1="52.07" y1="81.28" x2="36.83" y2="81.28" width="0.1524" layer="91"/>
<wire x1="36.83" y1="81.28" x2="36.83" y2="101.6" width="0.1524" layer="91"/>
<pinref part="FONTE1" gate="A" pin="14"/>
<wire x1="36.83" y1="101.6" x2="46.99" y2="101.6" width="0.1524" layer="91"/>
<pinref part="IGB1.1_DRIVER" gate="A" pin="6"/>
<wire x1="81.28" y1="109.22" x2="91.44" y2="109.22" width="0.1524" layer="91"/>
<wire x1="91.44" y1="109.22" x2="91.44" y2="101.6" width="0.1524" layer="91"/>
<junction x="91.44" y="101.6"/>
<pinref part="JP1" gate="A" pin="8"/>
<wire x1="59.69" y1="60.96" x2="59.69" y2="44.45" width="0.1524" layer="91"/>
<pinref part="JP1" gate="A" pin="9"/>
<wire x1="59.69" y1="44.45" x2="52.07" y2="44.45" width="0.1524" layer="91"/>
<wire x1="59.69" y1="60.96" x2="52.07" y2="60.96" width="0.1524" layer="91"/>
<wire x1="52.07" y1="60.96" x2="52.07" y2="81.28" width="0.1524" layer="91"/>
<junction x="52.07" y="81.28"/>
<wire x1="59.69" y1="44.45" x2="59.69" y2="31.75" width="0.1524" layer="91"/>
<wire x1="59.69" y1="31.75" x2="39.37" y2="31.75" width="0.1524" layer="91"/>
<junction x="59.69" y="44.45"/>
<wire x1="39.37" y1="31.75" x2="39.37" y2="41.91" width="0.1524" layer="91"/>
<wire x1="39.37" y1="41.91" x2="44.45" y2="41.91" width="0.1524" layer="91"/>
<wire x1="36.83" y1="101.6" x2="20.32" y2="101.6" width="0.1524" layer="91"/>
<wire x1="20.32" y1="101.6" x2="20.32" y2="153.67" width="0.1524" layer="91"/>
<junction x="36.83" y="101.6"/>
<wire x1="20.32" y1="153.67" x2="162.56" y2="153.67" width="0.1524" layer="91"/>
<wire x1="162.56" y1="153.67" x2="162.56" y2="129.54" width="0.1524" layer="91"/>
<wire x1="162.56" y1="129.54" x2="177.8" y2="129.54" width="0.1524" layer="91"/>
<wire x1="203.2" y1="101.6" x2="203.2" y2="81.28" width="0.1524" layer="91"/>
<pinref part="IGB2.1_DRIVER" gate="A" pin="3"/>
<wire x1="193.04" y1="101.6" x2="203.2" y2="101.6" width="0.1524" layer="91"/>
<wire x1="203.2" y1="81.28" x2="177.8" y2="81.28" width="0.1524" layer="91"/>
<pinref part="IGB2.1_DRIVER" gate="A" pin="6"/>
<wire x1="177.8" y1="81.28" x2="163.83" y2="81.28" width="0.1524" layer="91"/>
<wire x1="193.04" y1="109.22" x2="203.2" y2="109.22" width="0.1524" layer="91"/>
<wire x1="203.2" y1="109.22" x2="203.2" y2="101.6" width="0.1524" layer="91"/>
<junction x="203.2" y="101.6"/>
<pinref part="JP2" gate="A" pin="8"/>
<wire x1="171.45" y1="60.96" x2="171.45" y2="44.45" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="9"/>
<wire x1="171.45" y1="44.45" x2="163.83" y2="44.45" width="0.1524" layer="91"/>
<wire x1="171.45" y1="60.96" x2="163.83" y2="60.96" width="0.1524" layer="91"/>
<wire x1="163.83" y1="60.96" x2="163.83" y2="81.28" width="0.1524" layer="91"/>
<wire x1="171.45" y1="44.45" x2="171.45" y2="31.75" width="0.1524" layer="91"/>
<wire x1="171.45" y1="31.75" x2="151.13" y2="31.75" width="0.1524" layer="91"/>
<junction x="171.45" y="44.45"/>
<wire x1="151.13" y1="31.75" x2="151.13" y2="41.91" width="0.1524" layer="91"/>
<wire x1="151.13" y1="41.91" x2="156.21" y2="41.91" width="0.1524" layer="91"/>
<wire x1="177.8" y1="129.54" x2="177.8" y2="81.28" width="0.1524" layer="91"/>
<junction x="177.8" y="81.28"/>
<wire x1="281.94" y1="153.67" x2="281.94" y2="129.54" width="0.1524" layer="91"/>
<wire x1="281.94" y1="129.54" x2="297.18" y2="129.54" width="0.1524" layer="91"/>
<wire x1="322.58" y1="101.6" x2="322.58" y2="81.28" width="0.1524" layer="91"/>
<pinref part="IGB3.1_DRIVER" gate="A" pin="3"/>
<wire x1="312.42" y1="101.6" x2="322.58" y2="101.6" width="0.1524" layer="91"/>
<wire x1="322.58" y1="81.28" x2="297.18" y2="81.28" width="0.1524" layer="91"/>
<pinref part="IGB3.1_DRIVER" gate="A" pin="6"/>
<wire x1="297.18" y1="81.28" x2="283.21" y2="81.28" width="0.1524" layer="91"/>
<wire x1="312.42" y1="109.22" x2="322.58" y2="109.22" width="0.1524" layer="91"/>
<wire x1="322.58" y1="109.22" x2="322.58" y2="101.6" width="0.1524" layer="91"/>
<junction x="322.58" y="101.6"/>
<pinref part="JP3" gate="A" pin="8"/>
<wire x1="290.83" y1="60.96" x2="290.83" y2="44.45" width="0.1524" layer="91"/>
<pinref part="JP3" gate="A" pin="9"/>
<wire x1="290.83" y1="44.45" x2="283.21" y2="44.45" width="0.1524" layer="91"/>
<wire x1="290.83" y1="60.96" x2="283.21" y2="60.96" width="0.1524" layer="91"/>
<wire x1="283.21" y1="60.96" x2="283.21" y2="81.28" width="0.1524" layer="91"/>
<wire x1="290.83" y1="44.45" x2="290.83" y2="31.75" width="0.1524" layer="91"/>
<wire x1="290.83" y1="31.75" x2="270.51" y2="31.75" width="0.1524" layer="91"/>
<junction x="290.83" y="44.45"/>
<wire x1="270.51" y1="31.75" x2="270.51" y2="41.91" width="0.1524" layer="91"/>
<wire x1="270.51" y1="41.91" x2="275.59" y2="41.91" width="0.1524" layer="91"/>
<wire x1="297.18" y1="129.54" x2="297.18" y2="81.28" width="0.1524" layer="91"/>
<junction x="297.18" y="81.28"/>
<wire x1="162.56" y1="153.67" x2="281.94" y2="153.67" width="0.1524" layer="91"/>
<junction x="162.56" y="153.67"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="5"/>
<wire x1="81.28" y1="106.68" x2="90.17" y2="106.68" width="0.1524" layer="91"/>
<wire x1="90.17" y1="106.68" x2="90.17" y2="82.55" width="0.1524" layer="91"/>
<wire x1="90.17" y1="82.55" x2="50.8" y2="82.55" width="0.1524" layer="91"/>
<pinref part="JP1" gate="A" pin="4"/>
<wire x1="58.42" y1="59.69" x2="58.42" y2="49.53" width="0.1524" layer="91"/>
<wire x1="58.42" y1="49.53" x2="52.07" y2="49.53" width="0.1524" layer="91"/>
<wire x1="58.42" y1="59.69" x2="50.8" y2="59.69" width="0.1524" layer="91"/>
<wire x1="50.8" y1="59.69" x2="50.8" y2="82.55" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="IGB1.2_DRIVER" gate="A" pin="12"/>
<wire x1="81.28" y1="58.42" x2="91.44" y2="58.42" width="0.1524" layer="91"/>
<wire x1="91.44" y1="58.42" x2="91.44" y2="74.93" width="0.1524" layer="91"/>
<wire x1="91.44" y1="74.93" x2="49.53" y2="74.93" width="0.1524" layer="91"/>
<pinref part="JP1" gate="A" pin="2"/>
<wire x1="57.15" y1="58.42" x2="57.15" y2="52.07" width="0.1524" layer="91"/>
<wire x1="57.15" y1="52.07" x2="52.07" y2="52.07" width="0.1524" layer="91"/>
<wire x1="57.15" y1="58.42" x2="49.53" y2="58.42" width="0.1524" layer="91"/>
<wire x1="49.53" y1="58.42" x2="49.53" y2="74.93" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="4"/>
<wire x1="81.28" y1="104.14" x2="88.9" y2="104.14" width="0.1524" layer="91"/>
<wire x1="88.9" y1="104.14" x2="88.9" y2="83.82" width="0.1524" layer="91"/>
<wire x1="88.9" y1="83.82" x2="48.26" y2="83.82" width="0.1524" layer="91"/>
<pinref part="JP1" gate="A" pin="3"/>
<wire x1="39.37" y1="58.42" x2="39.37" y2="49.53" width="0.1524" layer="91"/>
<wire x1="39.37" y1="49.53" x2="44.45" y2="49.53" width="0.1524" layer="91"/>
<wire x1="39.37" y1="58.42" x2="48.26" y2="58.42" width="0.1524" layer="91"/>
<wire x1="48.26" y1="58.42" x2="48.26" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="FONTE1" gate="A" pin="1"/>
<wire x1="46.99" y1="134.62" x2="29.21" y2="134.62" width="0.1524" layer="91"/>
<wire x1="29.21" y1="134.62" x2="29.21" y2="142.24" width="0.1524" layer="91"/>
<wire x1="29.21" y1="142.24" x2="90.17" y2="142.24" width="0.1524" layer="91"/>
<wire x1="90.17" y1="142.24" x2="90.17" y2="119.38" width="0.1524" layer="91"/>
<pinref part="IGB1.1_DRIVER" gate="A" pin="10"/>
<wire x1="90.17" y1="119.38" x2="81.28" y2="119.38" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="FONTE1" gate="A" pin="2"/>
<wire x1="46.99" y1="132.08" x2="27.94" y2="132.08" width="0.1524" layer="91"/>
<wire x1="27.94" y1="132.08" x2="27.94" y2="143.51" width="0.1524" layer="91"/>
<wire x1="27.94" y1="143.51" x2="91.44" y2="143.51" width="0.1524" layer="91"/>
<wire x1="91.44" y1="143.51" x2="91.44" y2="116.84" width="0.1524" layer="91"/>
<pinref part="IGB1.1_DRIVER" gate="A" pin="9"/>
<wire x1="91.44" y1="116.84" x2="81.28" y2="116.84" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="IGB1.2_DRIVER" gate="A" pin="8"/>
<wire x1="81.28" y1="48.26" x2="91.44" y2="48.26" width="0.1524" layer="91"/>
<wire x1="91.44" y1="48.26" x2="91.44" y2="19.05" width="0.1524" layer="91"/>
<wire x1="91.44" y1="19.05" x2="27.94" y2="19.05" width="0.1524" layer="91"/>
<wire x1="27.94" y1="19.05" x2="27.94" y2="119.38" width="0.1524" layer="91"/>
<pinref part="FONTE1" gate="A" pin="7"/>
<wire x1="27.94" y1="119.38" x2="46.99" y2="119.38" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="IGB1.2_DRIVER" gate="A" pin="7"/>
<wire x1="81.28" y1="45.72" x2="90.17" y2="45.72" width="0.1524" layer="91"/>
<wire x1="90.17" y1="45.72" x2="90.17" y2="20.32" width="0.1524" layer="91"/>
<wire x1="90.17" y1="20.32" x2="29.21" y2="20.32" width="0.1524" layer="91"/>
<wire x1="29.21" y1="20.32" x2="29.21" y2="121.92" width="0.1524" layer="91"/>
<pinref part="FONTE1" gate="A" pin="6"/>
<wire x1="29.21" y1="121.92" x2="46.99" y2="121.92" width="0.1524" layer="91"/>
</segment>
</net>
<net name="DC+" class="0">
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="16"/>
<wire x1="81.28" y1="134.62" x2="86.36" y2="134.62" width="0.1524" layer="91"/>
<wire x1="86.36" y1="134.62" x2="86.36" y2="128.27" width="0.1524" layer="91"/>
<wire x1="86.36" y1="128.27" x2="102.87" y2="128.27" width="0.1524" layer="91"/>
<wire x1="102.87" y1="128.27" x2="102.87" y2="139.7" width="0.1524" layer="91"/>
<wire x1="102.87" y1="139.7" x2="110.49" y2="139.7" width="0.1524" layer="91"/>
<wire x1="7.62" y1="130.81" x2="16.51" y2="130.81" width="0.1524" layer="91"/>
<wire x1="16.51" y1="130.81" x2="16.51" y2="157.48" width="0.1524" layer="91"/>
<wire x1="16.51" y1="157.48" x2="110.49" y2="157.48" width="0.1524" layer="91"/>
<wire x1="110.49" y1="157.48" x2="110.49" y2="139.7" width="0.1524" layer="91"/>
<pinref part="IGB2.1_DRIVER" gate="A" pin="16"/>
<wire x1="193.04" y1="134.62" x2="198.12" y2="134.62" width="0.1524" layer="91"/>
<wire x1="198.12" y1="134.62" x2="198.12" y2="128.27" width="0.1524" layer="91"/>
<wire x1="198.12" y1="128.27" x2="214.63" y2="128.27" width="0.1524" layer="91"/>
<wire x1="214.63" y1="128.27" x2="214.63" y2="139.7" width="0.1524" layer="91"/>
<wire x1="214.63" y1="139.7" x2="222.25" y2="139.7" width="0.1524" layer="91"/>
<wire x1="222.25" y1="157.48" x2="222.25" y2="139.7" width="0.1524" layer="91"/>
<wire x1="110.49" y1="157.48" x2="132.08" y2="157.48" width="0.1524" layer="91"/>
<junction x="110.49" y="157.48"/>
<pinref part="IGB3.1_DRIVER" gate="A" pin="16"/>
<wire x1="132.08" y1="157.48" x2="222.25" y2="157.48" width="0.1524" layer="91"/>
<wire x1="312.42" y1="134.62" x2="317.5" y2="134.62" width="0.1524" layer="91"/>
<wire x1="317.5" y1="134.62" x2="317.5" y2="128.27" width="0.1524" layer="91"/>
<wire x1="317.5" y1="128.27" x2="334.01" y2="128.27" width="0.1524" layer="91"/>
<wire x1="334.01" y1="128.27" x2="334.01" y2="139.7" width="0.1524" layer="91"/>
<wire x1="334.01" y1="139.7" x2="341.63" y2="139.7" width="0.1524" layer="91"/>
<wire x1="341.63" y1="157.48" x2="341.63" y2="139.7" width="0.1524" layer="91"/>
<wire x1="222.25" y1="157.48" x2="246.38" y2="157.48" width="0.1524" layer="91"/>
<junction x="222.25" y="157.48"/>
<junction x="110.49" y="139.7"/>
<junction x="341.63" y="139.7"/>
<junction x="222.25" y="139.7"/>
<pinref part="U$5" gate="G$1" pin="C"/>
<pinref part="U$3" gate="G$1" pin="C"/>
<pinref part="U$1" gate="G$1" pin="C"/>
<wire x1="246.38" y1="157.48" x2="341.63" y2="157.48" width="0.1524" layer="91"/>
<wire x1="132.08" y1="87.63" x2="132.08" y2="157.48" width="0.1524" layer="91"/>
<junction x="132.08" y="157.48"/>
<wire x1="246.38" y1="87.63" x2="246.38" y2="157.48" width="0.1524" layer="91"/>
<junction x="246.38" y="157.48"/>
<wire x1="365.76" y1="87.63" x2="365.76" y2="157.48" width="0.1524" layer="91"/>
<wire x1="365.76" y1="157.48" x2="341.63" y2="157.48" width="0.1524" layer="91"/>
<junction x="341.63" y="157.48"/>
</segment>
</net>
<net name="VS1" class="0">
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="12"/>
<wire x1="81.28" y1="124.46" x2="86.36" y2="124.46" width="0.1524" layer="91"/>
<wire x1="86.36" y1="124.46" x2="86.36" y2="127" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="G"/>
<wire x1="86.36" y1="127" x2="100.33" y2="127" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VA" class="0">
<segment>
<pinref part="IGB1.1_DRIVER" gate="A" pin="11"/>
<wire x1="81.28" y1="121.92" x2="87.63" y2="121.92" width="0.1524" layer="91"/>
<wire x1="87.63" y1="121.92" x2="87.63" y2="125.73" width="0.1524" layer="91"/>
<wire x1="87.63" y1="125.73" x2="102.87" y2="125.73" width="0.1524" layer="91"/>
<wire x1="110.49" y1="119.38" x2="102.87" y2="119.38" width="0.1524" layer="91"/>
<wire x1="102.87" y1="119.38" x2="102.87" y2="125.73" width="0.1524" layer="91"/>
<pinref part="IGB1.2_DRIVER" gate="A" pin="1"/>
<wire x1="81.28" y1="30.48" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<wire x1="86.36" y1="30.48" x2="86.36" y2="41.91" width="0.1524" layer="91"/>
<wire x1="86.36" y1="41.91" x2="102.87" y2="41.91" width="0.1524" layer="91"/>
<wire x1="102.87" y1="41.91" x2="102.87" y2="53.34" width="0.1524" layer="91"/>
<wire x1="102.87" y1="53.34" x2="110.49" y2="53.34" width="0.1524" layer="91"/>
<wire x1="110.49" y1="119.38" x2="110.49" y2="83.82" width="0.1524" layer="91"/>
<wire x1="110.49" y1="83.82" x2="110.49" y2="53.34" width="0.1524" layer="91"/>
<wire x1="120.65" y1="83.82" x2="110.49" y2="83.82" width="0.1524" layer="91"/>
<junction x="110.49" y="83.82"/>
<junction x="110.49" y="53.34"/>
<junction x="110.49" y="119.38"/>
<pinref part="U$1" gate="G$1" pin="E"/>
<pinref part="U$2" gate="G$1" pin="C"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="IGB1.2_DRIVER" gate="A" pin="5"/>
<wire x1="81.28" y1="40.64" x2="100.33" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="G"/>
</segment>
</net>
<net name="DC-" class="0">
<segment>
<pinref part="IGB1.2_DRIVER" gate="A" pin="6"/>
<wire x1="81.28" y1="43.18" x2="87.63" y2="43.18" width="0.1524" layer="91"/>
<wire x1="87.63" y1="43.18" x2="87.63" y2="39.37" width="0.1524" layer="91"/>
<wire x1="87.63" y1="39.37" x2="102.87" y2="39.37" width="0.1524" layer="91"/>
<wire x1="102.87" y1="39.37" x2="102.87" y2="33.02" width="0.1524" layer="91"/>
<wire x1="102.87" y1="33.02" x2="110.49" y2="33.02" width="0.1524" layer="91"/>
<wire x1="8.89" y1="27.94" x2="16.51" y2="27.94" width="0.1524" layer="91"/>
<wire x1="16.51" y1="27.94" x2="16.51" y2="7.62" width="0.1524" layer="91"/>
<wire x1="16.51" y1="7.62" x2="110.49" y2="7.62" width="0.1524" layer="91"/>
<wire x1="110.49" y1="7.62" x2="110.49" y2="33.02" width="0.1524" layer="91"/>
<pinref part="IGB2.2_DRIVER" gate="A" pin="6"/>
<wire x1="193.04" y1="43.18" x2="199.39" y2="43.18" width="0.1524" layer="91"/>
<wire x1="199.39" y1="43.18" x2="199.39" y2="39.37" width="0.1524" layer="91"/>
<wire x1="199.39" y1="39.37" x2="214.63" y2="39.37" width="0.1524" layer="91"/>
<wire x1="214.63" y1="39.37" x2="214.63" y2="33.02" width="0.1524" layer="91"/>
<wire x1="214.63" y1="33.02" x2="222.25" y2="33.02" width="0.1524" layer="91"/>
<wire x1="222.25" y1="7.62" x2="222.25" y2="33.02" width="0.1524" layer="91"/>
<wire x1="110.49" y1="7.62" x2="132.08" y2="7.62" width="0.1524" layer="91"/>
<junction x="110.49" y="7.62"/>
<pinref part="IGB3.2_DRIVER" gate="A" pin="6"/>
<wire x1="132.08" y1="7.62" x2="222.25" y2="7.62" width="0.1524" layer="91"/>
<wire x1="312.42" y1="43.18" x2="318.77" y2="43.18" width="0.1524" layer="91"/>
<wire x1="318.77" y1="43.18" x2="318.77" y2="39.37" width="0.1524" layer="91"/>
<wire x1="318.77" y1="39.37" x2="334.01" y2="39.37" width="0.1524" layer="91"/>
<wire x1="334.01" y1="39.37" x2="334.01" y2="33.02" width="0.1524" layer="91"/>
<wire x1="334.01" y1="33.02" x2="341.63" y2="33.02" width="0.1524" layer="91"/>
<wire x1="341.63" y1="7.62" x2="341.63" y2="33.02" width="0.1524" layer="91"/>
<wire x1="222.25" y1="7.62" x2="246.38" y2="7.62" width="0.1524" layer="91"/>
<junction x="222.25" y="7.62"/>
<junction x="110.49" y="33.02"/>
<junction x="341.63" y="33.02"/>
<junction x="222.25" y="33.02"/>
<pinref part="U$6" gate="G$1" pin="E"/>
<pinref part="U$4" gate="G$1" pin="E"/>
<pinref part="U$2" gate="G$1" pin="E"/>
<wire x1="246.38" y1="7.62" x2="341.63" y2="7.62" width="0.1524" layer="91"/>
<wire x1="132.08" y1="80.01" x2="132.08" y2="7.62" width="0.1524" layer="91"/>
<junction x="132.08" y="7.62"/>
<wire x1="246.38" y1="7.62" x2="246.38" y2="80.01" width="0.1524" layer="91"/>
<junction x="246.38" y="7.62"/>
<wire x1="365.76" y1="80.01" x2="365.76" y2="7.62" width="0.1524" layer="91"/>
<wire x1="365.76" y1="7.62" x2="341.63" y2="7.62" width="0.1524" layer="91"/>
<junction x="341.63" y="7.62"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="IGB2.1_DRIVER" gate="A" pin="5"/>
<wire x1="193.04" y1="106.68" x2="201.93" y2="106.68" width="0.1524" layer="91"/>
<wire x1="201.93" y1="106.68" x2="201.93" y2="82.55" width="0.1524" layer="91"/>
<wire x1="201.93" y1="82.55" x2="162.56" y2="82.55" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="4"/>
<wire x1="170.18" y1="59.69" x2="170.18" y2="49.53" width="0.1524" layer="91"/>
<wire x1="170.18" y1="49.53" x2="163.83" y2="49.53" width="0.1524" layer="91"/>
<wire x1="170.18" y1="59.69" x2="162.56" y2="59.69" width="0.1524" layer="91"/>
<wire x1="162.56" y1="59.69" x2="162.56" y2="82.55" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="IGB2.2_DRIVER" gate="A" pin="12"/>
<wire x1="193.04" y1="58.42" x2="203.2" y2="58.42" width="0.1524" layer="91"/>
<wire x1="203.2" y1="58.42" x2="203.2" y2="74.93" width="0.1524" layer="91"/>
<wire x1="203.2" y1="74.93" x2="161.29" y2="74.93" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="2"/>
<wire x1="168.91" y1="58.42" x2="168.91" y2="52.07" width="0.1524" layer="91"/>
<wire x1="168.91" y1="52.07" x2="163.83" y2="52.07" width="0.1524" layer="91"/>
<wire x1="168.91" y1="58.42" x2="161.29" y2="58.42" width="0.1524" layer="91"/>
<wire x1="161.29" y1="58.42" x2="161.29" y2="74.93" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="IGB2.1_DRIVER" gate="A" pin="4"/>
<wire x1="193.04" y1="104.14" x2="200.66" y2="104.14" width="0.1524" layer="91"/>
<wire x1="200.66" y1="104.14" x2="200.66" y2="83.82" width="0.1524" layer="91"/>
<wire x1="200.66" y1="83.82" x2="160.02" y2="83.82" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="3"/>
<wire x1="151.13" y1="58.42" x2="151.13" y2="49.53" width="0.1524" layer="91"/>
<wire x1="151.13" y1="49.53" x2="156.21" y2="49.53" width="0.1524" layer="91"/>
<wire x1="151.13" y1="58.42" x2="160.02" y2="58.42" width="0.1524" layer="91"/>
<wire x1="160.02" y1="58.42" x2="160.02" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<wire x1="171.45" y1="142.24" x2="201.93" y2="142.24" width="0.1524" layer="91"/>
<wire x1="201.93" y1="142.24" x2="201.93" y2="119.38" width="0.1524" layer="91"/>
<pinref part="IGB2.1_DRIVER" gate="A" pin="10"/>
<wire x1="201.93" y1="119.38" x2="193.04" y2="119.38" width="0.1524" layer="91"/>
<pinref part="FONTE2.2" gate="A" pin="1"/>
<wire x1="163.83" y1="107.95" x2="171.45" y2="107.95" width="0.1524" layer="91"/>
<wire x1="171.45" y1="107.95" x2="171.45" y2="142.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<wire x1="170.18" y1="143.51" x2="203.2" y2="143.51" width="0.1524" layer="91"/>
<wire x1="203.2" y1="143.51" x2="203.2" y2="116.84" width="0.1524" layer="91"/>
<pinref part="IGB2.1_DRIVER" gate="A" pin="9"/>
<wire x1="203.2" y1="116.84" x2="193.04" y2="116.84" width="0.1524" layer="91"/>
<wire x1="170.18" y1="113.03" x2="170.18" y2="143.51" width="0.1524" layer="91"/>
<pinref part="FONTE2.2" gate="A" pin="3"/>
<wire x1="170.18" y1="113.03" x2="163.83" y2="113.03" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="IGB2.2_DRIVER" gate="A" pin="8"/>
<wire x1="193.04" y1="48.26" x2="203.2" y2="48.26" width="0.1524" layer="91"/>
<wire x1="203.2" y1="48.26" x2="203.2" y2="19.05" width="0.1524" layer="91"/>
<wire x1="203.2" y1="19.05" x2="139.7" y2="19.05" width="0.1524" layer="91"/>
<wire x1="139.7" y1="19.05" x2="139.7" y2="102.87" width="0.1524" layer="91"/>
<wire x1="149.86" y1="113.03" x2="149.86" y2="102.87" width="0.1524" layer="91"/>
<wire x1="149.86" y1="102.87" x2="139.7" y2="102.87" width="0.1524" layer="91"/>
<pinref part="FONTE2" gate="A" pin="4"/>
<wire x1="149.86" y1="113.03" x2="156.21" y2="113.03" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="IGB2.2_DRIVER" gate="A" pin="7"/>
<wire x1="193.04" y1="45.72" x2="201.93" y2="45.72" width="0.1524" layer="91"/>
<wire x1="201.93" y1="45.72" x2="201.93" y2="20.32" width="0.1524" layer="91"/>
<wire x1="201.93" y1="20.32" x2="140.97" y2="20.32" width="0.1524" layer="91"/>
<wire x1="140.97" y1="20.32" x2="140.97" y2="101.6" width="0.1524" layer="91"/>
<pinref part="FONTE2" gate="A" pin="6"/>
<wire x1="156.21" y1="107.95" x2="151.13" y2="107.95" width="0.1524" layer="91"/>
<wire x1="151.13" y1="107.95" x2="151.13" y2="101.6" width="0.1524" layer="91"/>
<wire x1="151.13" y1="101.6" x2="140.97" y2="101.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="IGB2.1_DRIVER" gate="A" pin="12"/>
<wire x1="193.04" y1="124.46" x2="198.12" y2="124.46" width="0.1524" layer="91"/>
<wire x1="198.12" y1="124.46" x2="198.12" y2="127" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="G"/>
<wire x1="198.12" y1="127" x2="212.09" y2="127" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VB" class="0">
<segment>
<pinref part="IGB2.1_DRIVER" gate="A" pin="11"/>
<wire x1="193.04" y1="121.92" x2="199.39" y2="121.92" width="0.1524" layer="91"/>
<wire x1="199.39" y1="121.92" x2="199.39" y2="125.73" width="0.1524" layer="91"/>
<wire x1="199.39" y1="125.73" x2="214.63" y2="125.73" width="0.1524" layer="91"/>
<wire x1="222.25" y1="119.38" x2="214.63" y2="119.38" width="0.1524" layer="91"/>
<wire x1="214.63" y1="119.38" x2="214.63" y2="125.73" width="0.1524" layer="91"/>
<pinref part="IGB2.2_DRIVER" gate="A" pin="1"/>
<wire x1="193.04" y1="30.48" x2="198.12" y2="30.48" width="0.1524" layer="91"/>
<wire x1="198.12" y1="30.48" x2="198.12" y2="41.91" width="0.1524" layer="91"/>
<wire x1="198.12" y1="41.91" x2="214.63" y2="41.91" width="0.1524" layer="91"/>
<wire x1="214.63" y1="41.91" x2="214.63" y2="53.34" width="0.1524" layer="91"/>
<wire x1="214.63" y1="53.34" x2="222.25" y2="53.34" width="0.1524" layer="91"/>
<wire x1="222.25" y1="119.38" x2="222.25" y2="83.82" width="0.1524" layer="91"/>
<wire x1="222.25" y1="83.82" x2="222.25" y2="53.34" width="0.1524" layer="91"/>
<wire x1="232.41" y1="83.82" x2="222.25" y2="83.82" width="0.1524" layer="91"/>
<junction x="222.25" y="83.82"/>
<junction x="222.25" y="53.34"/>
<junction x="222.25" y="119.38"/>
<pinref part="U$3" gate="G$1" pin="E"/>
<pinref part="U$4" gate="G$1" pin="C"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="IGB2.2_DRIVER" gate="A" pin="5"/>
<wire x1="193.04" y1="40.64" x2="212.09" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="G"/>
</segment>
</net>
<net name="TP2" class="0">
<segment>
<pinref part="FONTE1" gate="A" pin="13"/>
<wire x1="46.99" y1="104.14" x2="21.59" y2="104.14" width="0.1524" layer="91"/>
<wire x1="21.59" y1="104.14" x2="21.59" y2="152.4" width="0.1524" layer="91"/>
<wire x1="21.59" y1="152.4" x2="160.02" y2="152.4" width="0.1524" layer="91"/>
<pinref part="FONTE2" gate="A" pin="1"/>
<wire x1="151.13" y1="128.27" x2="151.13" y2="120.65" width="0.1524" layer="91"/>
<wire x1="151.13" y1="120.65" x2="156.21" y2="120.65" width="0.1524" layer="91"/>
<wire x1="160.02" y1="152.4" x2="160.02" y2="128.27" width="0.1524" layer="91"/>
<wire x1="160.02" y1="128.27" x2="151.13" y2="128.27" width="0.1524" layer="91"/>
<pinref part="FONTE3" gate="A" pin="1"/>
<wire x1="270.51" y1="128.27" x2="270.51" y2="120.65" width="0.1524" layer="91"/>
<wire x1="270.51" y1="120.65" x2="275.59" y2="120.65" width="0.1524" layer="91"/>
<wire x1="279.4" y1="152.4" x2="279.4" y2="128.27" width="0.1524" layer="91"/>
<wire x1="279.4" y1="128.27" x2="270.51" y2="128.27" width="0.1524" layer="91"/>
<wire x1="160.02" y1="152.4" x2="279.4" y2="152.4" width="0.1524" layer="91"/>
<junction x="160.02" y="152.4"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<pinref part="IGB3.1_DRIVER" gate="A" pin="5"/>
<wire x1="312.42" y1="106.68" x2="321.31" y2="106.68" width="0.1524" layer="91"/>
<wire x1="321.31" y1="106.68" x2="321.31" y2="82.55" width="0.1524" layer="91"/>
<wire x1="321.31" y1="82.55" x2="281.94" y2="82.55" width="0.1524" layer="91"/>
<pinref part="JP3" gate="A" pin="4"/>
<wire x1="289.56" y1="59.69" x2="289.56" y2="49.53" width="0.1524" layer="91"/>
<wire x1="289.56" y1="49.53" x2="283.21" y2="49.53" width="0.1524" layer="91"/>
<wire x1="289.56" y1="59.69" x2="281.94" y2="59.69" width="0.1524" layer="91"/>
<wire x1="281.94" y1="59.69" x2="281.94" y2="82.55" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<pinref part="IGB3.2_DRIVER" gate="A" pin="12"/>
<wire x1="312.42" y1="58.42" x2="322.58" y2="58.42" width="0.1524" layer="91"/>
<wire x1="322.58" y1="58.42" x2="322.58" y2="74.93" width="0.1524" layer="91"/>
<wire x1="322.58" y1="74.93" x2="280.67" y2="74.93" width="0.1524" layer="91"/>
<pinref part="JP3" gate="A" pin="2"/>
<wire x1="288.29" y1="58.42" x2="288.29" y2="52.07" width="0.1524" layer="91"/>
<wire x1="288.29" y1="52.07" x2="283.21" y2="52.07" width="0.1524" layer="91"/>
<wire x1="288.29" y1="58.42" x2="280.67" y2="58.42" width="0.1524" layer="91"/>
<wire x1="280.67" y1="58.42" x2="280.67" y2="74.93" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="IGB3.1_DRIVER" gate="A" pin="4"/>
<wire x1="312.42" y1="104.14" x2="320.04" y2="104.14" width="0.1524" layer="91"/>
<wire x1="320.04" y1="104.14" x2="320.04" y2="83.82" width="0.1524" layer="91"/>
<wire x1="320.04" y1="83.82" x2="279.4" y2="83.82" width="0.1524" layer="91"/>
<pinref part="JP3" gate="A" pin="3"/>
<wire x1="270.51" y1="58.42" x2="270.51" y2="49.53" width="0.1524" layer="91"/>
<wire x1="270.51" y1="49.53" x2="275.59" y2="49.53" width="0.1524" layer="91"/>
<wire x1="270.51" y1="58.42" x2="279.4" y2="58.42" width="0.1524" layer="91"/>
<wire x1="279.4" y1="58.42" x2="279.4" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<wire x1="290.83" y1="142.24" x2="321.31" y2="142.24" width="0.1524" layer="91"/>
<wire x1="321.31" y1="142.24" x2="321.31" y2="119.38" width="0.1524" layer="91"/>
<pinref part="IGB3.1_DRIVER" gate="A" pin="10"/>
<wire x1="321.31" y1="119.38" x2="312.42" y2="119.38" width="0.1524" layer="91"/>
<pinref part="FONTE3.2" gate="A" pin="1"/>
<wire x1="283.21" y1="107.95" x2="290.83" y2="107.95" width="0.1524" layer="91"/>
<wire x1="290.83" y1="107.95" x2="290.83" y2="142.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<wire x1="289.56" y1="143.51" x2="322.58" y2="143.51" width="0.1524" layer="91"/>
<wire x1="322.58" y1="143.51" x2="322.58" y2="116.84" width="0.1524" layer="91"/>
<pinref part="IGB3.1_DRIVER" gate="A" pin="9"/>
<wire x1="322.58" y1="116.84" x2="312.42" y2="116.84" width="0.1524" layer="91"/>
<pinref part="FONTE3.2" gate="A" pin="3"/>
<wire x1="289.56" y1="113.03" x2="289.56" y2="143.51" width="0.1524" layer="91"/>
<wire x1="283.21" y1="113.03" x2="289.56" y2="113.03" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="IGB3.2_DRIVER" gate="A" pin="8"/>
<wire x1="312.42" y1="48.26" x2="322.58" y2="48.26" width="0.1524" layer="91"/>
<wire x1="322.58" y1="48.26" x2="322.58" y2="19.05" width="0.1524" layer="91"/>
<wire x1="322.58" y1="19.05" x2="259.08" y2="19.05" width="0.1524" layer="91"/>
<wire x1="259.08" y1="19.05" x2="259.08" y2="102.87" width="0.1524" layer="91"/>
<wire x1="269.24" y1="113.03" x2="269.24" y2="102.87" width="0.1524" layer="91"/>
<wire x1="269.24" y1="102.87" x2="259.08" y2="102.87" width="0.1524" layer="91"/>
<pinref part="FONTE3" gate="A" pin="4"/>
<wire x1="269.24" y1="113.03" x2="275.59" y2="113.03" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$35" class="0">
<segment>
<pinref part="IGB3.2_DRIVER" gate="A" pin="7"/>
<wire x1="312.42" y1="45.72" x2="321.31" y2="45.72" width="0.1524" layer="91"/>
<wire x1="321.31" y1="45.72" x2="321.31" y2="20.32" width="0.1524" layer="91"/>
<wire x1="321.31" y1="20.32" x2="260.35" y2="20.32" width="0.1524" layer="91"/>
<wire x1="260.35" y1="20.32" x2="260.35" y2="101.6" width="0.1524" layer="91"/>
<pinref part="FONTE3" gate="A" pin="6"/>
<wire x1="275.59" y1="107.95" x2="270.51" y2="107.95" width="0.1524" layer="91"/>
<wire x1="270.51" y1="107.95" x2="270.51" y2="101.6" width="0.1524" layer="91"/>
<wire x1="270.51" y1="101.6" x2="260.35" y2="101.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<pinref part="IGB3.1_DRIVER" gate="A" pin="12"/>
<wire x1="312.42" y1="124.46" x2="317.5" y2="124.46" width="0.1524" layer="91"/>
<wire x1="317.5" y1="124.46" x2="317.5" y2="127" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="G"/>
<wire x1="317.5" y1="127" x2="331.47" y2="127" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VC" class="0">
<segment>
<pinref part="IGB3.1_DRIVER" gate="A" pin="11"/>
<wire x1="312.42" y1="121.92" x2="318.77" y2="121.92" width="0.1524" layer="91"/>
<wire x1="318.77" y1="121.92" x2="318.77" y2="125.73" width="0.1524" layer="91"/>
<wire x1="318.77" y1="125.73" x2="334.01" y2="125.73" width="0.1524" layer="91"/>
<wire x1="341.63" y1="119.38" x2="334.01" y2="119.38" width="0.1524" layer="91"/>
<wire x1="334.01" y1="119.38" x2="334.01" y2="125.73" width="0.1524" layer="91"/>
<pinref part="IGB3.2_DRIVER" gate="A" pin="1"/>
<wire x1="312.42" y1="30.48" x2="317.5" y2="30.48" width="0.1524" layer="91"/>
<wire x1="317.5" y1="30.48" x2="317.5" y2="41.91" width="0.1524" layer="91"/>
<wire x1="317.5" y1="41.91" x2="334.01" y2="41.91" width="0.1524" layer="91"/>
<wire x1="334.01" y1="41.91" x2="334.01" y2="53.34" width="0.1524" layer="91"/>
<wire x1="334.01" y1="53.34" x2="341.63" y2="53.34" width="0.1524" layer="91"/>
<wire x1="341.63" y1="119.38" x2="341.63" y2="83.82" width="0.1524" layer="91"/>
<wire x1="341.63" y1="83.82" x2="341.63" y2="53.34" width="0.1524" layer="91"/>
<wire x1="351.79" y1="83.82" x2="341.63" y2="83.82" width="0.1524" layer="91"/>
<junction x="341.63" y="83.82"/>
<junction x="341.63" y="119.38"/>
<junction x="341.63" y="53.34"/>
<pinref part="U$5" gate="G$1" pin="E"/>
<pinref part="U$6" gate="G$1" pin="C"/>
</segment>
</net>
<net name="N$38" class="0">
<segment>
<pinref part="IGB3.2_DRIVER" gate="A" pin="5"/>
<wire x1="312.42" y1="40.64" x2="331.47" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$6" gate="G$1" pin="G"/>
</segment>
</net>
<net name="TP1" class="0">
<segment>
<pinref part="FONTE3.2" gate="A" pin="6"/>
<wire x1="283.21" y1="120.65" x2="288.29" y2="120.65" width="0.1524" layer="91"/>
<wire x1="288.29" y1="120.65" x2="288.29" y2="128.27" width="0.1524" layer="91"/>
<wire x1="288.29" y1="128.27" x2="280.67" y2="128.27" width="0.1524" layer="91"/>
<wire x1="280.67" y1="128.27" x2="280.67" y2="151.13" width="0.1524" layer="91"/>
<pinref part="FONTE2.2" gate="A" pin="6"/>
<wire x1="163.83" y1="120.65" x2="168.91" y2="120.65" width="0.1524" layer="91"/>
<wire x1="168.91" y1="120.65" x2="168.91" y2="128.27" width="0.1524" layer="91"/>
<wire x1="161.29" y1="151.13" x2="22.86" y2="151.13" width="0.1524" layer="91"/>
<wire x1="22.86" y1="151.13" x2="22.86" y2="106.68" width="0.1524" layer="91"/>
<pinref part="FONTE1" gate="A" pin="12"/>
<wire x1="22.86" y1="106.68" x2="46.99" y2="106.68" width="0.1524" layer="91"/>
<wire x1="168.91" y1="128.27" x2="161.29" y2="128.27" width="0.1524" layer="91"/>
<wire x1="161.29" y1="128.27" x2="161.29" y2="151.13" width="0.1524" layer="91"/>
<wire x1="280.67" y1="151.13" x2="161.29" y2="151.13" width="0.1524" layer="91"/>
<junction x="161.29" y="151.13"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
