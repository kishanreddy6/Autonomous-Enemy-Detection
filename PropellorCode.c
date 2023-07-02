#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
#include "ping.h"                             // Include ping header

const int trigPin_front = 8;                       //Assign Trigpin to 8
const int echoPin_front = 9;                         //Assign echopin to 9
const int trigPin_left = 12;                  //Assign Trigpin to 11
const int echoPin_left = 11;                  //Assign echopin to 12
int right_servo=16, left_servo=17;            // Assign pin16 to right servo. Assign pin17 to left servo. 
int right_forward=-30, left_forward=30;       //assign right pin to -40 for forward direction for 40% speed 
int right_backward=30, left_backward=-30;
int distance_front;
int distance_left;
int first_intersection = 1;
int intersections;
int countmid=0;
int b_line;
int b_int=0;
int a_int;
int m_int;
int l_line;
int m_line;
int m_line1;
int k=0;
int k1=0;
int k2 =0;
int z=0;
int cint;
int stop = 0;
const int camera=14;
int sensor1, sensor2, sensor3, sensor4, sensor5, sensor6 ,sensor7, sensor8;
int a=1000;

int g_int;
int obstacle_found = 0;
int obstacleback_found = 0;
int linesense();
int ultrasense();
int ultrasense_left();
int linefollow();
int objectb6();
int i;
int l;
int b_intersection;
int a_intersection;
int count_object;
int mid_counter;
int back_counter = 0;
int new_intersections=0;
int rnew_intersections=0;
int l_line1 = 0;
int b1_int = 0;

//unsigned int stack[40+100];
//static volatile int cog2;
//void camerafeed();

int main()                                    // Main function
{
  while(1)
  {
   // cog2=cogstart(&//camerafeed, NULL, stack, sizeof(stack));
    linesense();
    linefollow();
    
    
    if(sensor1<=a && sensor2<=a && sensor3<=a && sensor4<=a && sensor5<=a && sensor6<=a && sensor7<=a && sensor8<=a)            //IR on black line move forward
    {
    //stop both the Motors
    servo_speed(right_servo, 0);                          
    servo_speed(left_servo, 0);
    }
    
    
                                                     
    else if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
    {        
      if (first_intersection == 1)
      {
         freqout(10, 1000, 3000);
         servo_speed(right_servo, right_forward);                          
         servo_speed(left_servo, left_forward);
         pause(500);
         first_intersection=0;
         intersections=0;
         linefollow(); 
         b_line=0;
       } 
       
      else if(b_line==0)   //take left at i1
      {
        freqout(10, 1000, 3000);
        //print("entered loop second \n");
        pause(10);
        intersections ++;
        //print("taking left at i1 \n");
        servo_speed(right_servo, right_forward);                          
        servo_speed(left_servo, left_backward);
        pause(1500);
        linefollow(); 
        b_line=1;
       }
       
       else if(b_line==1)   //taking right at b1
       {
        print("taking right at b1 \n");
        freqout(10, 1000, 1750);
        servo_speed(right_servo, right_backward);                          
        servo_speed(left_servo, left_forward);
        pause(1450);
        b_line=5;
        b_int=1;
        object_sensing();
        linefollow();
       }
                   
            
       else if(b_int==1)//sense objects on b lane
       {
       //print(" b-INT \n");
       linefollow(); 
        while (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && k<=2)
        {
         linesense();
         //print("reached b intersection %d \n", k);
         freqout(10, 1000, 1000);
         object_sensing();
         linefollow();
         k=k+1;
         print("%d \n", k);                   
         }
         
         linefollow();  
         if(k==2)
            {
            cint = 0;
            b_int=4;
            //object_sensing();
            linefollow();
            } 
        }                          
          
       else if (cint==0)
       {
       object_sensing(); 
       freqout(10, 1000, 3000);
       servo_speed(right_servo, right_forward);                          
       servo_speed(left_servo, left_forward);
       pause(150);   //take right       
       servo_speed(right_servo, right_backward);                          
       servo_speed(left_servo, left_forward);
       pause(1500);   //take right
       linesense();
       //servo_speed(right_servo, right_forward);                          
       //servo_speed(left_servo, left_forward);
       print("going towards line A \n");
       linefollow();
       cint=2;
       }       
        
       else if(cint==2)
       {
       while (k1<1)         //from b4 towards A
       {
         linesense();
         if (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && k1<2)
         {
           k1=k1+1;
           freqout(10, 1000, 3000);
           servo_speed(right_servo, right_forward);                          
           servo_speed(left_servo, left_forward);
           linefollow();
           
           //print("Intersection Crossed = %d \n", k1); 
           if (k1==2)  
           {
           cint=3;
           break;
           }
           //freqout(10, 1000, 1000); 
         }
         
         linefollow();
       }
       cint=3;
       }                  
                  
       else if(cint==3)
       {
         freqout(10, 1000, 3000);
         servo_speed(right_servo, right_backward);                          
         servo_speed(left_servo, left_forward);
         pause(1450);   //take right at A lane
         print("taking right at A lane\n");
         //object_sensing(); //for checking I removed this line 
         a_int = 1;
         cint = 6;
         linefollow();     
       }
               
     else if(a_int==1)
     {    
         while (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && l<=3)
         {
         linesense();
         print("reached a intersection %d \n", l);
         freqout(10, 1000, 1000);
         object_sensing();
         linefollow();
         l=l+1;
         print("%d \n", l);                   
         }
         linefollow();  
         
         if(l==3)
         {
         servo_speed(right_servo, right_backward);                          
         servo_speed(left_servo, left_forward);
         pause(1500);   //take right towards mid lane
         print("taking right towards  mid lane\n");
         a_int=2;
         }          
       }
       
       //loop for mid intersection turn right
       else if(a_int==2)
       {
         linefollow();
         //print("wait for obstacle sensing 2000)\n");
         //pause(2000);
         //
         //obstacle_sensing();//not required else it will go to b5 direction because of flag  
         
         if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && a_int==2)
         {
           
           print("Intersection Mid---\n");
           freqout(10, 1000, 3000);
           servo_speed(right_servo, right_backward);                          
           servo_speed(left_servo, left_forward);
           pause(1500);   //take right
           //print("taking right \n");
           linefollow(); 
           obstacle_sensing();
                   
           }
           
           //obstacle_sensing(); //not required else it will go to b5 direction because of flag
         }
         
        //entered mid  line and sensing for obstacle
         else if(a_int==3)
          {
             //obstacle_sensing();
             linefollow();
             while (obstacle_found != 1){
               
               //linefollow();
               if (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && a_int==3)  
               {
                mid_counter++;
                freqout(10, 1000, 3000);
                linesense();
                print("a_int %d\n",a_int);
                print("mid counter numbers %d\n", mid_counter);
                
               }
               
                  
               obstacle_sensing();
               
               
               }
             
             if(count_object<50 && obstacle_found == 1)
             {
               a_int=40;
          
             }
     

            
             
             
             
             
          }
          
          
       else if(a_int==40){
             
 
            print("Searching object8 fn\n");
            object_sensing();

                    
           //check counter to go to i1
           
           while(new_intersections<(mid_counter))
           {
             linesense();
             linefollow();
             object_sensing();
             if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
             {
              freqout(10, 1000, 3000);
              servo_speed(right_servo, right_forward);                          
              servo_speed(left_servo, left_forward);
              //pause(500);
              print("new intersection value %d\n",new_intersections);
              new_intersections++;
              linefollow();
             } 
            
             else
             {
              linefollow(); 
              object_sensing();
             } 
             a_int=400;
             l_line1=1; 
             linesense();
             linefollow();
             
            }//end of while loop
            
            a_int=400;
            l_line1=1; 
            linesense();
            linefollow();
            
         }            
    
          else if(l_line1==1)   //take right at i1
        {
        freqout(10, 1000, 1500);
        linesense();
        print("entering lane Mid to B to sense object at b5\n");
        //pause(10);
        //intersections ++;
        print("taking right at i1 \n");
        servo_speed(right_servo, right_backward);                          
        servo_speed(left_servo, left_forward);
        pause(1500); 
        
        l_line1=100;
       }
  
 else if (l_line1==100){
   linefollow();
   print("Going towards b5 \n");
   
   if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a){
     
     b1_int=1;
     print("taking right at b1 again \n");
     freqout(10, 1000, 1750);
     servo_speed(right_servo, right_backward);                          
     servo_speed(left_servo, left_forward);
     pause(1450);
     l_line1 = 101;
     
     }
     
     else{
       print("Going towards b5 else \n");
       linesense();
       linefollow();
   }
   
  }
  
 else if(l_line1 == 101){
    if(mid_counter==3)
    {
      if (count_object==50)
      {
         high(15);
        pause(100);
        low(15); 
        high(15);
        pause(100);
        low(15);
        servo_speed(right_servo, 0);                          
        servo_speed(left_servo, 0);
        high(15);
        print("Completed run\n");
        pause(15000);
        stop = 1;
        ultrasense();
       }
    
    
    
      else if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
       {
        linesense();
        freqout(10, 1000, 1000);
        linefollow();
        object_sensing();
       
      }
      
      else 
      {
        linefollow();
        object_sensing();
       }
    }   
    
    else if(mid_counter<3){
     while (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && z<=2)
        {
         linesense();
         print("going in side intersection %d \n", z);
         freqout(10, 1000, 1000);
         //object_sensing();
         linefollow();
         z=z+1;
         print("%d \n", z);                   
         }
         
         //linefollow();  
         if(z==2)
            {
            cint = 99;
            l_line1 = 1001;
            print("breaking z2\n");
            //linefollow();
            } 
       }
                                
 }          
     else if (cint==99)
       { 
       freqout(10, 1000, 3000);          
       servo_speed(right_servo, right_backward);                          
       servo_speed(left_servo, left_forward);
       pause(1500);   //take right
       linesense();
       //servo_speed(right_servo, right_forward);                          
       //servo_speed(left_servo, left_forward);
       print("going towards line A \n");
       linefollow();
       cint=98;
       }       
                      
                  
       else if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && cint==98)
       {
         freqout(10, 1000, 3000);
         servo_speed(right_servo, right_backward);                          
         servo_speed(left_servo, left_forward);
         pause(1450);   //take right at A lane
         print("taking right at A lane\n");
         
         //a_int = 1;
         //cint = 6;
         linefollow();
         object_sensing();
         obstacle_sensing_back();

         cint = 97;     
       } 
       
      /*
     else if (cint==97){
       
         object_sensing();
         obstacle_sensing_back();
         cint = 96;
         print("Going to mint 40 \n");
         m_int = 40;
     
     }
      */
    
 
     else if(m_int==40)
          {
            print("Searching object8 from back fn\n");
            object_sensing();
                   
           //check counter to go to i1
           
           while(rnew_intersections<(back_counter+1))
           {
             linesense();
             linefollow();
             object_sensing();
             if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
             {
              freqout(10, 1000, 3000);
              servo_speed(right_servo, right_forward);                          
              servo_speed(left_servo, left_forward);
              //pause(500);
              print("reverse new intersection value %d\n",rnew_intersections);
              rnew_intersections++;
              linefollow();
             } 
            
             else
             {
              linefollow(); 
              object_sensing();
             } 
                   
      
             linesense();
             linefollow();
             
            }//end of while loop
            
            m_int=400;
            m_line1=2; 
            linesense();
            linefollow();
            
         }            
    
     else if(m_line1==2) {
      
      print("Taking uturn at i5 \n");
      freqout(10, 1000, 3000); 
      servo_speed(right_servo, right_backward);                          
      servo_speed(left_servo, left_forward);
      pause(3500);
      linefollow();
      object_sensing();
      m_line1=3;
   }     

    else if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a && m_line1==3)
       {
         print("Taking uturn again at i4\n");
        freqout(10, 1000, 3000); 
        servo_speed(right_servo, right_backward);                          
        servo_speed(left_servo, left_forward);
        pause(3500);
        linefollow();
        object_sensing(); 
        m_line1 = 1;    
         
       }
      
      
      
      
      else if(m_line1==1)   //take left at i5
      {
        if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
       {
         freqout(10, 1000, 1500);
        linesense();
        print("taking left at i5 \n");
        servo_speed(right_servo, right_forward);                          
        servo_speed(left_servo, left_backward);
        pause(1500);
        linefollow();
        m_line1=100;
       }
        
    } 
  
     else if (m_line1==100){
     linesense();
     linefollow();
     print("Going towards b5 \n");
    linefollow();

    if(sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)
    {
     
     b1_int=1;
     print("taking right at b5 \n");
     freqout(10, 1000, 1750);
     servo_speed(right_servo, right_backward);                          
     servo_speed(left_servo, left_forward);
     pause(1450);
     object_sensing();

     m_line1 = 101;
     
     }
     
     else{
       print("Going towards b5 \n");
       linesense();
       linefollow();
   }
   
  }
      
     
      
 //end of back object sensing     
            
    else{
        linefollow();
        object_sensing();
        } 
 
    }

     

          else if(stop ==1)
          {  
          print("Run completed");
          high(15);
          pause(100);
          low(15); 
          high(15);
          pause(100);
          low(15);
          servo_speed(right_servo, 0);                          
          servo_speed(left_servo, 0);
          high(15);
          print("Completed run\n");
          pause(15000);
          stop = 1;
          ultrasense();
          }                             
                                            
            

     
   else
   {
   linefollow(); 
   object_sensing();
   } 
   //} //removed to check                                
} //end of while loop
}  //end of main


/* --------------------------------------------------------------------------------------------------------------------------------------------*/
                                          //functions
                                            
                                            
//Line sensing function
int linesense()
{ 
    set_directions(7, 0, 0b11111111); // Set P0-P7 as output pins.
    set_outputs(7, 0, 0b11111111); // Set P0-P7 as high pins  
    pause(1);    
    sensor1 = rc_time(0, 1);           
    sensor2 = rc_time(1, 1);       
    sensor3 = rc_time(2, 1);      
    sensor4 = rc_time(3, 1);      
    sensor5 = rc_time(4, 1);      
    sensor6 = rc_time(5, 1);      
    sensor7 = rc_time(6, 1);      
    sensor8 = rc_time(7, 1); 
    //printf(" %d ,%d ,%d ,%d ,%d ,%d ,%d , %d \n", sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8);   
    //pause(10);       
}

int ultrasense()
{
 low(trigPin_front);
 pulse_out(trigPin_front, 10);
 long tEcho = pulse_in(echoPin_front, 1);
 distance_front = tEcho / 58;
 //print("%d cm\n", distance_front);
 //pause(200);
}


int ultrasense_left()
{
 low(trigPin_left);
 pulse_out(trigPin_left, 10);
 long tEcho = pulse_in(echoPin_left, 1);
 distance_left = tEcho / 58;
 //print("%d cm\n", distance_left);
 //pause(200);
}

int linefollow()
{
   linesense();
  
    if(sensor1<=a && sensor3>=a && sensor4>=a && sensor8<=a)            //IR on black line move forward
    {
    //Move both the Motors
    servo_speed(right_servo, right_forward);                          
    servo_speed(left_servo, left_forward);
    }
                                                     
    else if((sensor2<=a || sensor3<=a) && (sensor4>=a||sensor5>=a))
    {
    // stopping the left wheel and move left wheel for moving left SIDE
    servo_speed(right_servo, right_forward);                          
    servo_speed(left_servo, 0);
    }
                                                        //IR not fully on black line: perform correction action move Left
    else if((sensor2>=a || sensor3>=a) && (sensor4<=a || sensor5<=a))  
    {
    //stopping the right wheel and move left wheel for moving right SIDE
    servo_speed(right_servo, 0);                          
    servo_speed(left_servo, left_forward);
    } 
    
    else 
    {
    //Move both the Motors
    servo_speed(right_servo, right_forward);                          
    servo_speed(left_servo, left_forward);
    }
}  

void object_sensing()
{
  ultrasense_left();
  //print("my distance %d cm\n", distance_left);
  //print("Sensing object left \n");
  
  if (distance_left<=9)
   {
    servo_speed(right_servo, 0);                          
    servo_speed(left_servo, 0);
    pause(2000);
    high(14);    //Giving input for Raspberry Pi to make enemy/friendlies detection and perform action.
    pause(1000);
    low(14);
    pause(10);
    servo_speed(right_servo, right_forward);                          
    servo_speed(left_servo, left_forward);
    pause(1000);
    
    
    //print("object detected at %d cm\n", distance_left);
    //pause(200);
    count_object++; 
    //print("object count %d cm\n", count_object);
      
      if (count_object==50 && obstacle_found==1)
      {
        high(15);
        pause(100);
        low(15); 
        high(15);
        pause(100);
        low(15);
        servo_speed(right_servo, 0);                          
        servo_speed(left_servo, 0);
        high(15);
        print("Completed run\n");
        pause(15000);
        stop = 1;
        l_line1=105;
        ultrasense();
               
       }      
    } 
}

void obstacle_sensing_midline(){
            object_sensing();
            linefollow();
             while (obstacle_found != 1){
               
               linefollow();
               if (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)  
               {
                mid_counter++;
                freqout(10, 1000, 3000);
                linesense();
                print("a_int %d\n",a_int);
                print("mid counter numbers %d\n", mid_counter);
                
               }
               
                  
               obstacle_sensing();
               
               
               }
             
             if(count_object<50 && obstacle_found == 1)
             {
               a_int=40;
          
             }
  
  
  
 }


void obstacle_sensing_midline2()
{
            object_sensing();
            linefollow();
             while (obstacleback_found != 1){
               
               linefollow();
               if (sensor1>=a && sensor2>=a && sensor3>=a && sensor4>=a && sensor5>=a && sensor6>=a && sensor7>=a && sensor8>=a)  
               {
                back_counter++;
                freqout(10, 1000, 3000);
                linesense();
                print("a_int %d\n",a_int);
                print("back_counter numbers %d\n", back_counter);
                
               }
               
                  
               obstacle_sensing_back();
               
               
               }
             
             if(count_object<50 && obstacleback_found == 1)
             {
               //cint = 50;
               m_int=40;
          
             }
  
  
  
 }




void obstacle_sensing()
{
  ultrasense();
  print("my distance %d cm\n", distance_front);
  print("Sensing object front");
  
  if (distance_front<=13) 
   {
     if (distance_front<=13 && count_object==50)  
     {
      high(15);
      pause(100);
      low(15); 
      high(15);
      pause(100);
      low(15);
      servo_speed(right_servo, 0);                          
      servo_speed(left_servo, 0);
      high(15);
      print("Completed run\n");
      pause(15000);
      stop = 1;
      ultrasense();
      }
      
   
    else if(distance_front<=13)
    {
      print("Mid counter is %d\n", mid_counter);
      obstacle_found = 1;
      ultrasense();
      high(15);
      pause(100);
      low(15);
      print("Obstacle Found %d\n", mid_counter);
       
       //  take U-turn
          
      print("Taking uturn \n");
      freqout(10, 1000, 3000); 
      servo_speed(right_servo, right_backward);                          
      servo_speed(left_servo, left_forward);
      pause(3500); 
    
      a_int=40;
       
    }
    
           
   }
   
   else if (a_int!=40)
   {
     print("Going to obstacle_sensing_midline \n");
     obstacle_sensing_midline();
   }
   
 }
 

void obstacle_sensing_back()
{
  
  ultrasense();
  print("my distance %d cm\n", distance_front);
  print("Sensing object front");
  
  if (distance_front<=13) 
   {
     if (distance_front<=13 && count_object==50)  
     {
      high(15);
      pause(100);
      low(15); 
      high(15);
      pause(100);
      low(15);
      servo_speed(right_servo, 0);                          
      servo_speed(left_servo, 0);
      high(15);
      print("Completed run\n");
      pause(15000);
      stop = 1;
      ultrasense();
      }
      
                
    else if(distance_front<=13)
    {
      print("Back counter is %d\n", back_counter);
      obstacleback_found = 1;
      ultrasense();
      high(15);
      pause(100);
      low(15);
      print("Obstacle Found %d\n", back_counter);
       
       //  take U-turn
          
      print("Taking uturn \n");
      freqout(10, 1000, 3000); 
      servo_speed(right_servo, right_backward);                          
      servo_speed(left_servo, left_forward);
      pause(3500); 
      cint = 50;
      m_int=40;
       
    }
    
           
   }
   
   else if (m_int!=40)
   {
     print("Going to obstacle_sensing_midline2 \n");
     obstacle_sensing_midline2();
   }
   

   
 }     