-------------------------------------------------------------------------------
--  Simple ATC Radar Simulator
--  File: atc_simulator.adb
--  Author: Tisha Patel
--  Date: June 2025
--  Description: Main simulation procedure for ATC radar system.
-------------------------------------------------------------------------------

with Ada.Text_IO;         use Ada.Text_IO;
with Ada.Float_Text_IO;   use Ada.Float_Text_IO;
with Ada.Integer_Text_IO; use Ada.Integer_Text_IO;
with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;


procedure ATC_Simulator is
   -- Simulation parameters
   Num_Aircraft : constant Integer := 3;
   Steps        : constant Integer := 10;
   Min_Sep_NM   : constant Float := 5.0;   -- Min horizontal separation (nm)
   Min_Sep_Ft   : constant Integer := 1000; -- Min vertical separation (ft)

   -- Aircraft tracking types
   type Aircraft_ID is range 1 .. Num_Aircraft;
   type Position is record
      X, Y : Float;   -- Position in nautical miles
      Alt  : Integer;  -- Altitude in feet
   end record;

   type Aircraft is record
      Call_Sign : String(1..8);  -- Flight identifier
      Pos       : Position;       -- Current position
      Heading   : Float;          -- Degrees (0-359)
      Speed     : Float;          -- Knots
   end record;

   type Aircraft_Array is array (Aircraft_ID) of Aircraft;

   -- Initial aircraft states
   Planes : Aircraft_Array := (
      ("AAL123  ", (0.0, 0.0, 30000), 90.0, 450.0),
      ("UAL456  ", (10.0, 5.0, 31000), 270.0, 430.0),
      ("DAL789  ", (20.0, -10.0, 30000), 180.0, 400.0)
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

   -- Conflict prediction check
   function Predict_Conflict(A1, A2 : Aircraft) return Boolean is
   begin
      return Distance(A1.Pos, A2.Pos) < Min_Sep_NM
        and abs(A1.Pos.Alt - A2.Pos.Alt) < Min_Sep_Ft;
   end Predict_Conflict;

   -- Display aircraft state
   procedure Print_Aircraft(A : Aircraft) is
   begin
      Put(A.Call_Sign & " Pos:(");
      Put(A.Pos.X, Fore => 1, Aft => 1, Exp => 0); Put(",");
      Put(A.Pos.Y, Fore => 1, Aft => 1, Exp => 0); Put(") ");
      Put("Alt:"); Put(A.Pos.Alt); Put(" ");
      Put("Hdg:"); Put(A.Heading, Fore => 1, Aft => 0, Exp => 0); Put(" ");
      Put("Spd:"); Put(A.Speed, Fore => 1, Aft => 0, Exp => 0);
      New_Line;
   end Print_Aircraft;

begin
   Put_Line("=== ATC Radar Simulator ===");
   for Step in 1 .. Steps loop
      Put_Line("--- Time Step " & Integer'Image(Step) & " ---");

      -- Update all aircraft positions
      for I in Aircraft_ID loop
         declare
            Dist : constant Float := Planes(I).Speed / 60.0; -- Distance per minute
            Rad  : constant Float := Deg_To_Rad(Planes(I).Heading);
         begin
            Planes(I).Pos.X := Planes(I).Pos.X + Dist * Cos(Rad);
            Planes(I).Pos.Y := Planes(I).Pos.Y + Dist * Sin(Rad);
         end;
         Print_Aircraft(Planes(I));
      end loop;

      -- Conflict detection
      for I in Aircraft_ID loop
         for J in Aircraft_ID loop
            if I < J and then Predict_Conflict(Planes(I), Planes(J)) then
               Put_Line("!! CONFLICT: " & Planes(I).Call_Sign &
                        " and " & Planes(J).Call_Sign);
               Put_Line("   ADVISORY: " & Planes(I).Call_Sign &
                        " climb 2000ft, " & Planes(J).Call_Sign &
                        " turn right 30°");
            end if;
         end loop;
      end loop;

      delay 1.0; -- Simulate real-time update
   end loop;
   Put_Line("=== Simulation Complete ===");
end ATC_Simulator;
