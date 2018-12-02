/* 
  ArbotiX Test Program for use with PyPose 0015
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
 
#include <ax12.h>
#include <BioloidController.h>
#include <Motors2.h>

BioloidController bioloid = BioloidController(1000000);
Motors2 drive = Motors2();

#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_LOAD_SEQ    9
#define ARB_PLAY_SEQ    10
#define ARB_LOOP_SEQ    11
#define ARB_TEST        25

int mode = 0;                   // where we are in the frame

unsigned char id = 0;           // id of this frame
unsigned char length = 0;       // length of this frame
unsigned char ins = 0;          // instruction of this frame

unsigned char params[143];      // parameters (match RX-64 buffer size)
unsigned char index = 0;        // index in param buffer

int checksum;                   // checksum

typedef struct{
    unsigned char pose;         // index of pose to transition to 
    int time;                   // time for transition
} sp_trans_t;

//  pose and sequence storage
int poses[30][AX12_MAX_SERVOS]; // poses [index][servo_id-1]
sp_trans_t sequence[50];        // sequence
int seqPos;                     // step in current sequence

void setup(){
    Serial.begin(38400); 
    drive.init();   
    pinMode(0,OUTPUT);          // status LED
}

/* 
 * packet: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 *
 * ID = 253 for these special commands!
 * Pose Size = 7, followed by single param: size of pose
 * Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
 * Seq Size = 9, followed by single param: size of seq
 * Load Seq = A, followed by index/times (# of parameters = 3*seq_size) 
 * Play Seq = B, no params
 * Loop Seq = C, 
 */

void loop(){
    int i;
    
    // process messages
    while(Serial.available() > 0){
        // We need to 0xFF at start of packet
        if(mode == 0){         // start of new packet
            if(Serial.read() == 0xff){
                mode = 2;
                digitalWrite(0,HIGH-digitalRead(0));
            }
        //}else if(mode == 1){   // another start byte
        //    if(Serial.read() == 0xff)
        //        mode = 2;
        //    else
        //        mode = 0;
        }else if(mode == 2){   // next byte is index of servo
            id = Serial.read();    
            if(id != 0xff)
                mode = 3;
        }else if(mode == 3){   // next byte is length
            length = Serial.read();
            checksum = id + length;
            mode = 4;
        }else if(mode == 4){   // next byte is instruction
            ins = Serial.read();
            checksum += ins;
            index = 0;
            mode = 5;
        }else if(mode == 5){   // read data in 
            params[index] = Serial.read();
            checksum += (int) params[index];
            index++;
            if(index + 1 == length){  // we've read params & checksum
                mode = 0;
                if((checksum%256) != 255){ 
                    // return a packet: FF FF id Len Err params=None check
                    Serial.write(0xff);
                    Serial.write(0xff);
                    Serial.write(id);
                    Serial.write(2);
                    Serial.write(64);
                    Serial.write(255-((66+id)%256));
                }else{
                    if(id == 253){
                        // return a packet: FF FF id Len Err params=None check
                        Serial.write(0xff);
                        Serial.write(0xff);
                        Serial.write(id);
                        Serial.write(2);
                        Serial.write((unsigned char)0);
                        Serial.write(255-((2+id)%256));
                        // special ArbotiX instructions
                        // Pose Size = 7, followed by single param: size of pose
                        // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
                        // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
                        // Play Seq = A, no params
                        if(ins == ARB_SIZE_POSE){
                            bioloid.poseSize = params[0];
                            bioloid.readPose();    
                            //Serial.println(bioloid.poseSize);
                        }else if(ins == ARB_LOAD_POSE){
                            int i;    
                            Serial.print("New Pose:");
                            for(i=0; i<bioloid.poseSize; i++){
                                poses[params[0]][i] = params[(2*i)+1]+(params[(2*i)+2]<<8); 
                                //Serial.print(poses[params[0]][i]);
                                //Serial.print(",");     
                            } 
                            Serial.println("");
                        }else if(ins == ARB_LOAD_SEQ){
                            int i;
                            for(i=0;i<(length-2)/3;i++){
                                sequence[i].pose = params[(i*3)];
                                sequence[i].time = params[(i*3)+1] + (params[(i*3)+2]<<8);
                                //Serial.print("New Transition:");
                                //Serial.print((int)sequence[i].pose);
                                //Serial.print(" in ");
                                //Serial.println(sequence[i].time);      
                            }
                        }else if(ins == ARB_PLAY_SEQ){
                            seqPos = 0;
                            while(sequence[seqPos].pose != 0xff){
                                int i;
                                int p = sequence[seqPos].pose;
                                // are we HALT?
                                if(Serial.read() == 'H') return;
                                // load pose
                                for(i=0; i<bioloid.poseSize; i++){
                                    bioloid.setNextPose(i+1,poses[p][i]);
                                } 
                                // interpolate
                                bioloid.interpolateSetup(sequence[seqPos].time);
                                while(bioloid.interpolating)
                                    bioloid.interpolateStep();
                                // next transition
                                seqPos++;
                            }
                        }else if(ins == ARB_LOOP_SEQ){
                            while(1){
                                seqPos = 0;
                                while(sequence[seqPos].pose != 0xff){
                                    int i;
                                    int p = sequence[seqPos].pose;
                                    // are we HALT?
                                    if(Serial.read() == 'H') return;
                                    // load pose
                                    for(i=0; i<bioloid.poseSize; i++){
                                        bioloid.setNextPose(i+1,poses[p][i]);
                                    } 
                                    // interpolate
                                    bioloid.interpolateSetup(sequence[seqPos].time);
                                    while(bioloid.interpolating)
                                        bioloid.interpolateStep();
                                    // next transition
                                    seqPos++;
                                }
                            }
                        }else if(ins == ARB_TEST){
                            int i;
                            // Test Digital I/O
                            for(i=0;i<8;i++){
                                // test digital
                                pinMode(i,OUTPUT);
                                digitalWrite(i,HIGH);  
                                // test analog
                                pinMode(31-i,OUTPUT);
                                digitalWrite(31-i,HIGH);
                                
                                delay(500);
                                digitalWrite(i,LOW);
                                digitalWrite(31-i,LOW);
                            }
                            // Test Ax-12
                            for(i=452;i<552;i+=20){
                                SetPosition(1,i);
                                delay(200);
                            }
                            // Test Motors
                            drive.set(-255,-255);
                            delay(500);
                            drive.set(0,0);
                            delay(1500);
                            drive.set(255,255);
                            delay(500);
                            drive.set(0,0);
                            delay(1500);
                            // Test Analog I/O
                            for(i=0;i<8;i++){
                                // test digital
                                pinMode(i,OUTPUT);
                                digitalWrite(i,HIGH);  
                                // test analog
                                pinMode(31-i,OUTPUT);
                                digitalWrite(31-i,HIGH);
                                
                                delay(500);
                                digitalWrite(i,LOW);
                                digitalWrite(31-i,LOW);
                            }
                        }   
                    }else{
                        int i;
                        // pass thru
                       if(ins == AX_READ_DATA){
                            int i;
                            ax12GetRegister(id, params[0], params[1]);
                            // return a packet: FF FF id Len Err params check
                            if(ax_rx_buffer[3] > 0){
                            for(i=0;i<ax_rx_buffer[3]+4;i++)
                                Serial.write(ax_rx_buffer[i]);
                            }
                            ax_rx_buffer[3] = 0;
                        }else if(ins == AX_WRITE_DATA){
                            if(length == 4){
                                ax12SetRegister(id, params[0], params[1]);
                            }else{
                                int x = params[1] + (params[2]<<8);
                                ax12SetRegister2(id, params[0], x);
                            }
                            // return a packet: FF FF id Len Err params check
                            Serial.write(0xff);
                            Serial.write(0xff);
                            Serial.write(id);
                            Serial.write(2);
                            Serial.write((unsigned char)0);
                            Serial.write(255-((2+id)%256));
                        }
                    }
                }
            }
        }
    }
    
    // update joints
    bioloid.interpolateStep();
}

