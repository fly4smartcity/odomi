string name

uint8 START=0
uint8 STOP=1
uint8 TAKE_OFF=2
uint8 LAND=3
uint8 GO_TO=4
uint8 HOVER=5
uint8 CIRCLE=6
uint8 HEAD_TO=7
#enumerated above
uint8 type

Action[] pre_actions
Action[] post_actions

#filled only in case of TakeOff and Circle moves
float64 altitude

#filled only in case of Goto or Circle move
Coordinate target_position

uint8 DIRECT=0
uint8 HORIZONTAL_FIRST=1
uint8 VERTICAL_FIRST=2
#enumerated above, filled only in case of GoTo move
uint8 strategy

#filled only in case of Hover or Circle moves
duration duration

#following parameters are filled only in case Circle move
float64 radius
bool clockwise


#filled only in case of HeadTo move
float64 direction

