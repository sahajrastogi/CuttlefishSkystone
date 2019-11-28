package org.firstinspires.ftc.teamcode;

public class ButtonOp {
   public boolean curr;
   public boolean prev;

   public ButtonOp(){
       curr = false;
       prev = false;
   }

   public boolean isPressed(){
       return curr;
   }

   public boolean onRelease(){
       return (!curr && prev);
   }

   public boolean onPress(){
       return (curr && !prev);
   }

   public void update(boolean a){
       prev = curr;
       curr = a;
   }
}
