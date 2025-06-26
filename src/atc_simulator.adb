-------------------------------------------------------------------------------
--  Simple ATC Radar Simulator
--  File: atc_simulator.adb
--  Author: Tisha Patel
--  Date: June 2025
--  Description: Main simulation procedure for ATC radar system.
--        Added: Advanced ATC radar system with comprehensive state management.
--  Tracks 3 aircrafts over 10 minutes
--  Monitors aircraft positions, speeds, altitudes, and flight phases while enforcing
--  ICAO separation standards and wake turbulence rules for aviation safety.
--
--  Features:
--  > Real-time position tracking with 1-minute radar sweeps
--  > RVSM separation enforcement (1000ft above FL290)
--  > Wake turbulence category management (Heavy/Medium/Light aircraft)
--  > Flight phase monitoring (Taxi/Takeoff/Climb/Cruise/Descent/Approach/Landing)
--  > Conflict detection and advisory generation
--  > Comprehensive aircraft state validation
--
--  Aircraft Simulated:
--  > AAL123: Heavy aircraft cruising at FL300, heading East
--  > UAL456: Medium aircraft descending FL305 => FL260, heading West
--  > DAL789: Heavy aircraft climbing FL300 => FL420, heading South
-------------------------------------------------------------------------------

with Ada.Text_IO;         use Ada.Text_IO;
with Ada.Float_Text_IO;   use Ada.Float_Text_IO;
with Ada.Integer_Text_IO; use Ada.Integer_Text_IO;
with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;
with Ada.Exceptions;      use Ada.Exceptions;
with Ada.Calendar;        use Ada.Calendar;


procedure ATC_Simulator is

   --Aviation Specific types with safety
   subtype Altitude_Ft is Integer range 0 .. 60_000 ;
   subtype Heading_Deg is Float range 0.0 .. 359.9;
   subtype Speed_Knots is Float range 0.0 .. 1000.0 ;
   subtype Coordinate_NM is Float range -1000.0 .. 1000.0;
   subtype Call_Sign_Type is String(1..8);


   --Flight Levels and categories  enumeration type , a user-defined type
   type Flight_Level is (VFR , IFR_LOW , IFR_HIGH , RVSM);
   type Aircraft_Category is (LIGHT , MEDIUM , HEAVY , SUPER);
   type Flight_Phase is (TAXI , TAKEOFF , CLIMB , CRUISE , DESCENT , APPROACH , LANDING);

   --Position with validation
   type Position is record
      X, Y: Coordinate_NM;   -- Position in Nautical miles
      Alt : Altitude_Ft;     -- Altitude in feet
   end record;

   --Aircraft state  with comprehensive tracking
   type  Aircraft_State is record
      Call_Sign : Call_Sign_Type;
      Pos       : Position;
      Heading   : Heading_Deg;
      Speed     : Speed_Knots;
      Category  : Aircraft_Category;
      Phase     : Flight_Phase;
      Last_Contact: Time;
      Squawk    : String(1..4);
      Vertical_Rate : Integer range -6000 .. 6000;   -- ft/min
   end record;

   -- Simulation parameters
   Num_Aircraft : constant Integer := 3;
   Steps        : constant Integer := 10;

   --Separation standards (ICAO)
   Min_Sep_NM     : constant Float := 5.0;    -- Min horizontal separation (nm)
   Min_Sep_Ft     : constant Integer := 1000; -- Min vertical separation (ft)
   RVSM_Sep_Ft    : constant Integer :=1000;  -- RVSM separation
   Wake_Sep_Heavy : constant Float :=6.0 ;    -- Wake turbulence separation

   -- Aircraft tracking types
   type Aircraft_ID is range 1 .. Num_Aircraft;
   type Aircraft_Array is array (Aircraft_ID) of Aircraft_State;

   --Custom Exceptions for aviation safety
   Aircraft_Conflict_Error : exception;
   Altitude_Violation_Error : exception;
   Speed_Violation_Error : exception;


   -- Initial aircraft states
   Planes : Aircraft_Array := (
      ("AAL123  ", (0.0, 0.0, 30000), 90.0, 450.0     , HEAVY  , CRUISE  , Clock,  "1234" , 0 ),
      ("UAL456  ", (10.0, 5.0, 31000), 270.0, 430.0   , MEDIUM , DESCENT , Clock,  "4321" , -500),
      ("DAL789  ", (20.0, -10.0, 30000), 180.0, 400.0 , HEAVY  , CLIMB   , Clock,  "3456" , 1200)
   );

   -- Convert degrees to radians
   function Deg_To_Rad(D : Float) return Float is
      Pi : constant Float := 3.14159265;
   begin
      return D * Pi / 180.0;
   end Deg_To_Rad;

   -- Calculate 2D distance between positions
   function Distance(P1, P2 : Position) return Float is
      DX : Float := P1.X - P2.X;
      DY : Float := P1.Y - P2.Y;
   begin
      return Sqrt(DX*DX + DY*DY);
   end Distance;

   --calculate required separation
   function Required_Separation(Cat1 , Cat2 : Aircraft_Category) return Float is
   begin
      case Cat1 is
         when SUPER => return 10.0 ;
         when HEAVY =>
            case Cat2 is
               when LIGHT => return Wake_Sep_Heavy;
               when others => return Min_Sep_NM;
            end case;
         when others => return Min_Sep_NM;
      end case;
   end Required_Separation;


   -- Conflict prediction check with wake turbulence
   function Predict_Conflict(A1, A2 : Aircraft_State) return Boolean is
      Horizontal_Sep : Float := Distance(A1.Pos , A2.Pos);
      Vertical_Sep   : Integer := abs(A1.Pos.Alt - A2.Pos.Alt);
      Required_H_Sep  : Float := Required_Separation(A1.Category , A2.Category);
      Required_V_Sep  : Integer := (if A1.Pos.Alt >= 29000 and A2.Pos.Alt >=29000
                                   then RVSM_Sep_Ft else Min_Sep_Ft);
   begin
      return Horizontal_Sep < Required_H_Sep
        and abs Vertical_Sep < Required_V_Sep;
   end Predict_Conflict;

   -- Validate aircraft state for safety
   procedure Validate_Aircraft_State(A : in out  Aircraft_State) is
   begin
      --check alt constraints first
      if A.Pos.Alt < 0 or A.Pos.Alt >60000 then
         raise Altitude_Violation_Error with
           "Altitude" & Integer'Image(A.Pos.Alt) & "Outside safe limits for " & A.Call_Sign;
      end if;

      --Check speed
      if A.Speed < 0.0 or A.Speed >1000.0 then
         raise Speed_Violation_Error with
           "Speed" & Float'Image(A.Speed) & "Outside safe limits for " & A.Call_Sign ;
      end if;

      -- Normalize heading
      while A.Heading >= 360.0 loop
         A.Heading := A.Heading - 360.0 ;
      end loop;
      while A.Heading < 0.0 loop
         A.Heading := A.Heading + 360.0 ;
      end loop;

      --Update last contact time
      A.Last_Contact := Clock ;
   end Validate_Aircraft_State;

   -- Displaying Aircraft with Full information now
   procedure Display_Aircraft_state(A: Aircraft_State) is

      function Category_String (Cat : Aircraft_Category) return String is
      begin
         case Cat is
            when LIGHT  => return "L";
            when MEDIUM => return "M";
            when HEAVY  => return "H";
            when SUPER  => return "S";
         end case ;
      end Category_String;

      function Phase_String(Phase : Flight_Phase) return String is
      begin
         case Phase is
            when TAXI => return "TAXI";
            when TAKEOFF  => return "T/O ";
            when CLIMB    => return "CLB ";
            when CRUISE   => return "CRZ ";
            when DESCENT  => return "DES ";
            when APPROACH => return "APP ";
            when LANDING  => return "LND ";
         end case;
      end Phase_String;

   begin
      Put(A.Call_Sign & " [" & A.Squawk & "] ");
      Put( "(" & Category_String(A.Category) & ")" ) ;
      Put( Phase_String(A.Phase) & " " );
      Put("Pos: (");
      Put(A.Pos.X, Fore => 2, Aft => 1, Exp => 0); Put(",");
      Put(A.Pos.Y, Fore => 2, Aft => 1, Exp => 0); Put(") ");
      Put("FL"); Put(A.Pos.Alt / 100 , Width => 3); Put(" ");

      --Put("Alt:"); Put(A.Pos.Alt); Put(" ");
      Put("HDG:"); Put(A.Heading, Fore => 3, Aft => 0, Exp => 0); Put("deg ");
      Put("SPD:"); Put(A.Speed, Fore => 3, Aft => 0, Exp => 0); Put("kt ");
      if A.Vertical_Rate /= 0 then
         Put("V/S:"); Put(A.Vertical_Rate, Width => 5); Put("fpm ");
      end if;

      New_Line;
   end Display_Aircraft_state;


   --ISSUE CONFLICT RESOLUTION
   procedure Issue_Advisory(A1, A2 : Aircraft_State) is
   begin
      Put_Line("*** TRAFFIC CONFLICT ALERT ***");
      Put_Line("Aircraft: " & A1.Call_Sign & " and " & A2.Call_Sign);
      Put_Line("Separation: " & Float'Image(Distance(A1.Pos, A2.Pos)) & "nm horizontal, " &
               Integer'Image(abs(A1.Pos.Alt - A2.Pos.Alt)) & "ft vertical");

      -- Issue altitude-based resolution
      if A1.Pos.Alt < A2.Pos.Alt then
         Put_Line("ADVISORY: " & A1.Call_Sign & " - DESCEND AND MAINTAIN FL" &
                  Integer'Image((A1.Pos.Alt - 2000) / 100));
         Put_Line("ADVISORY: " & A2.Call_Sign & " - CLIMB AND MAINTAIN FL" &
                  Integer'Image((A2.Pos.Alt + 2000) / 100));
      else
         Put_Line("ADVISORY: " & A1.Call_Sign & " - CLIMB AND MAINTAIN FL" &
                  Integer'Image((A1.Pos.Alt + 2000) / 100));
         Put_Line("ADVISORY: " & A2.Call_Sign & " - DESCEND AND MAINTAIN FL" &
                  Integer'Image((A2.Pos.Alt - 2000) / 100));
      end if;
      Put_Line("");
   end Issue_Advisory;

   --update aircraft position with realistic movement
    procedure Update_Aircraft_Position(A : in out Aircraft_State; Delta_Time_Min : Float) is
      Distance_NM : Float := A.Speed * Delta_Time_Min / 60.0;
      Heading_Rad : Float := Deg_To_Rad(A.Heading);
      Altitude_Change : Integer := Integer(Float(A.Vertical_Rate) * Delta_Time_Min);
   begin
      -- Update horizontal position
      A.Pos.X := A.Pos.X + Distance_NM * Cos(Heading_Rad);
      A.Pos.Y := A.Pos.Y + Distance_NM * Sin(Heading_Rad);

      -- Update altitude
      A.Pos.Alt := A.Pos.Alt + Altitude_Change;

      -- Validate the new state
      Validate_Aircraft_State(A);
   exception
      when E : others =>
         Put_Line("ERROR updating " & A.Call_Sign & ": " & Exception_Message(E));
   end Update_Aircraft_Position;


begin
   Put_Line("=== ATC Radar Simulator ===");
   Put_Line("Monitoring " & Integer'Image(Num_Aircraft) & " Aircrafts-");
   Put_Line("-ICAO separation standards enforced");
   Put_Line("-Wake turbulence categories active");
   Put_Line("");

   for Step in 1 .. Steps loop
      Put_Line("===RADAR SWEEP " & Integer'Image(Step) & "===");
      Put_Line("--- Time Step " & Duration'Image(Duration(Step)) & " minutes");

      begin
      -- Update all aircraft positions

      for I in Aircraft_ID loop
         Update_Aircraft_Position(Planes(I) , 1.0) ;     -- At 1 min intervals
         Display_Aircraft_state(Planes(I));
      end loop;
         Put_Line (" ");

         -- Better Conflict detection
         declare
            Conflicts_Detected: Boolean := False;

         begin
            for I in Aircraft_ID loop
               for J in Aircraft_ID loop
                  if I < J and then Predict_Conflict(Planes(I) , Planes(J)) then
                     Issue_Advisory(Planes(I) , Planes(J));
                     Conflicts_Detected := True;
                  end if;
               end loop;
            end loop;

            if not Conflicts_Detected then
               Put_Line(" All aircraft maintaining safe separation");
            end if;
         end;

      exception
         when E : Aircraft_Conflict_Error => Put_Line("CRITICAL: " & Exception_Message(E));
         when E : others =>  Put_Line("SYSTEM ERROR: " & Exception_Message(E));
      end;

      Put_Line("---");

      delay 1.0; -- Simulate real-time update
   end loop;
   Put_Line("=== Simulation Complete ===");
end ATC_Simulator;
