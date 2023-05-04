/*基础方位路径如下*/
/* 0 1 2
   3 p 4
   5 6 7*/  
/*如需改变地图尺寸则需修改宏定义中的row和column的值*/ 
/*如需改变程序运行速度则需改变如下程序中两个Sleep函数中参数的值*/
/*此程序并不支持入口点不在原点的情况*/
/*-------------------------------------------------------------------------------*/
#include<stdio.h>
#include<stdlib.h>
#include<windows.h>
#include<string.h>

#define robot_position 'R'
#define row 18
#define column 56 //根据地图尺寸大小改变参数的值 

int Obligatory_temp = 0; //必须到达目标点个数统计 
int Necessary_temp = 0;//可到达的目标点 
char Direction_temp[10000] = {0}; //方位暂存数组 
int temp = 0;//方位暂存数组下标 
char Map_temp[row][column];//地图元素暂存数组 

void Map_Init();
void Map_show(); //函数声明 

struct Robot_Nature{
	int Axis[2]; //机器人所在位置结构体 
}Robot;

struct Necessary_Target{
	char Direction[10000]; //相对于原点的方位路径 
	int Axis[2]; //目标点坐标 
	int Distance; //离原点的距离 
}Necessary_Net[20];

struct Obligatory_Target{
	char Direction[10000];
	int Axis[2];
	int Distance;
}Obligatory_Net[20];

struct Map{
	char nature; //该点的属性 
	int Axis[2]; //该点在图上的坐标 
	struct Map *Next_Element[8];//它周围可能的最多八个下一个节点 
	struct Map *Last_Element; //它的上一个节点 
	int Last_Element_Mark;
	int Invisited ; //是否被访问过 
	
}Map_Element[row][column],*Entrance = &Map_Element[0][0]; /*初始化地图元素结构体*/


void Check_Round_FindFather(struct Map *p,int flag){
	int Axis[2] = {p->Axis[0],p->Axis[1]}; //读取坐标 
	for(int i=0 ;i<8;i++){ /*判断下一个节点不是墙且没有被访问过且该点在地图上是存在的，若满足条件，标记为可到达的下一个节点*/ 
	   if(i == 0) p->Next_Element[i] = ((Axis[0]-1>=0&&Axis[0]-1<=row-1)&&(Axis[1]-1>=0&&Axis[1]-1<=column-1))?((Map_Element[Axis[0]-1][Axis[1]-1].Invisited != 1&&Map_Element[Axis[0]-1][Axis[1]-1].nature!='.'&&Map_Element[Axis[0]-1][Axis[1]-1].Last_Element == NULL)? &Map_Element[Axis[0]-1][Axis[1]-1]:NULL):NULL;
		if(i == 1) p->Next_Element[i] = ((Axis[0]-1>=0&&Axis[0]-1<=row-1)&&(Axis[1]>=0&&Axis[1]<=column-1))?((Map_Element[Axis[0]-1][Axis[1]].Invisited != 1&&Map_Element[Axis[0]-1][Axis[1]].nature!='.'&&Map_Element[Axis[0]-1][Axis[1]].Last_Element == NULL)? &Map_Element[Axis[0]-1][Axis[1]]:NULL):NULL;
		if(i == 2) p->Next_Element[i] = ((Axis[0]-1>=0&&Axis[0]-1<=row-1)&&(Axis[1]+1>=0&&Axis[1]+1<=column-1))?((Map_Element[Axis[0]-1][Axis[1]+1].Invisited != 1&&Map_Element[Axis[0]-1][Axis[1]+1].nature!='.'&&Map_Element[Axis[0]-1][Axis[1]+1].Last_Element == NULL)? &Map_Element[Axis[0]-1][Axis[1]+1]:NULL):NULL;
		if(i == 3) p->Next_Element[i] = ((Axis[0]>=0&&Axis[0]<=row-1)&&(Axis[1]-1>=0&&Axis[1]-1<=column-1))?((Map_Element[Axis[0]][Axis[1]-1].Invisited != 1&&Map_Element[Axis[0]][Axis[1]-1].nature!='.'&&Map_Element[Axis[0]][Axis[1]-1].Last_Element == NULL)? &Map_Element[Axis[0]][Axis[1]-1]:NULL):NULL;
		if(i == 4) p->Next_Element[i] = ((Axis[0]>=0&&Axis[0]<=row-1)&&(Axis[1]+1>=0&&Axis[1]+1<=column-1))?((Map_Element[Axis[0]][Axis[1]+1].Invisited != 1&&Map_Element[Axis[0]][Axis[1]+1].nature!='.'&&Map_Element[Axis[0]][Axis[1]+1].Last_Element == NULL)? &Map_Element[Axis[0]][Axis[1]+1]:NULL):NULL;
		if(i == 5) p->Next_Element[i] = ((Axis[0]+1>=0&&Axis[0]+1<=row-1)&&(Axis[1]-1>=0&&Axis[1]-1<=column-1))?((Map_Element[Axis[0]+1][Axis[1]-1].Invisited != 1&&Map_Element[Axis[0]+1][Axis[1]-1].nature!='.'&&Map_Element[Axis[0]+1][Axis[1]-1].Last_Element == NULL)? &Map_Element[Axis[0]+1][Axis[1]-1]:NULL):NULL;
		if(i == 6) p->Next_Element[i] = ((Axis[0]+1>=0&&Axis[0]+1<=row-1)&&(Axis[1]>=0&&Axis[1]<=column-1))?((Map_Element[Axis[0]+1][Axis[1]].Invisited != 1&&Map_Element[Axis[0]+1][Axis[1]].nature!='.'&&Map_Element[Axis[0]+1][Axis[1]].Last_Element == NULL)? &Map_Element[Axis[0]+1][Axis[1]]:NULL):NULL;
		if(i == 7) p->Next_Element[i] = ((Axis[0]+1>=0&&Axis[0]+1<=row-1)&&(Axis[1]+1>=0&&Axis[1]+1<=column-1))?((Map_Element[Axis[0]+1][Axis[1]+1].Invisited != 1&&Map_Element[Axis[0]+1][Axis[1]+1].nature!='.'&&Map_Element[Axis[0]+1][Axis[1]+1].Last_Element == NULL)? &Map_Element[Axis[0]+1][Axis[1]+1]:NULL):NULL;
		if(p->Next_Element[i]!=NULL&&flag == 1){
			p->Next_Element[i]->Last_Element = p; //标记可到达的下一个节点 的上一个节点（即该节点本身） 
			p->Next_Element[i]->Last_Element_Mark = i;
		}
	}

	
}

/*需要注意的是，当两个节点拥有同一个子节点时，后读取的父节点将低子节点进行强行夺点，即可能存在先前节点的子节点缺失情况发生*/

int Check_NULL(struct Map *s){ //检查是否有下一个节点，如果有的话返回1，否则返回0 
	for(int i = 0;i < 8; i++){
		if(s->Next_Element[i]!=NULL){
			return 1;
		}
	}
	return 0;
}

void Visit(struct Map *p){
	Direction_temp[temp] = p->Last_Element_Mark + '0'; //记录当前路径 
	Direction_temp[++temp] = '\0';
	p->Invisited = 1;  //已访问过 
	Check_Round_FindFather(p,1); //检查该节点周围并寻找上一个节点 
	
	if(p->nature == '*'){ //如果是Findable target 
		strcpy(Necessary_Net[Necessary_temp].Direction,Direction_temp); //复制当前路径到目标点路径 
		Necessary_Net[Necessary_temp].Axis[0] = p->Axis[0];  //记录坐标 
		Necessary_Net[Necessary_temp].Axis[1] = p->Axis[1]; 
		Necessary_Net[Necessary_temp].Distance = strlen(Direction_temp) - 1; //记录长度。由于头结点不算作总长度故需-1 
		Necessary_temp++;
	}
	if(p->nature == '#'){ //如果是Obligatory target 
		strcpy(Obligatory_Net[Obligatory_temp].Direction,Direction_temp);
		Obligatory_Net[Obligatory_temp].Axis[0] = p->Axis[0];
		Obligatory_Net[Obligatory_temp].Axis[1] = p->Axis[1];
		Obligatory_Net[Obligatory_temp].Distance = strlen(Direction_temp) - 1;
		Obligatory_temp++;
	}
	
	Map_Init();
	Map_temp[p->Axis[0]][p->Axis[1]] = robot_position;
	Sleep(100); /*更改参数调整运行速度*/
	system("cls");
	Map_show();
	/*初始化附地图暂存数组，地图清屏更新，呈现下一帧画面，机器人方位移动*/
}

void Go_Back_FatherNet(struct Map *p){
	
	int flag = 0; //返回结束标记 
	while(flag == 0){
		for(int i = 0;i<8;i++){
			if(p->Next_Element[i]==NULL){
				
				continue; //如果当前节点无子节点 
			}else{
				flag = 1;
				break;//如果有的话就结束 
			}
		}
		if(flag == 0){
			Direction_temp[--temp] = '\0'; //如果当前节点无子节点，则清除当前路径的末位 
			Map_Init();
			Map_temp[p->Last_Element->Axis[0]][p->Last_Element->Axis[1]] = robot_position;
			Sleep(100);/*更改参数调整运行速度*/
			system("cls");
			Map_show();
			/*更新画面*/
			p->Last_Element->Next_Element[p->Last_Element_Mark] = NULL; //将前一个节点指向当前节点的指针设为空 
			p = p->Last_Element; //迭代到上一个节点 
		}
	}
	
}

void Map_Save(){
	/*载入地图*/
	FILE *fp;
	fp = fopen("Map1.txt","r"); //如需读取其他的地图需更改地图文件名称 
	char ch;
	int i = 0;
	int j = 0;
	while(fp != NULL&&!(i==row - 1&&j==column)){
		(Map_Element[i][j]).nature = fgetc(fp);
		printf("%c",(Map_Element[i][j]).nature);
		Map_Element[i][j].Axis[0] = i;
		Map_Element[i][j].Axis[1] = j;
		if(j!=column){
			j++;
			continue;
		}else{
			
			i++;
			j = 0;
			continue;
		}
		
	}
	fclose(fp);
}
void Map_Init(){
	for(int i = 0 ; i<row ; i++) for(int j=0 ;j < column ; j++) {
		Map_temp[i][j] = Map_Element[i][j].nature;
	}
}

void Map_show(){ /*输出当前地图*/
	for(int i = 0 ; i<row ; i++){
		for(int j = 0; j < column ; j++){
			printf("%c",Map_temp[i][j]);
		}
		printf("\n");
	}
	printf("\n");
	if(Necessary_temp != 0){
		for(int i = 0;i<Necessary_temp ;i++){
			printf("Findable Target[%d] Has Been Found !\n",i+1);
			printf("Axis: (%d, %d)  ",Necessary_Net[i].Axis[0],Necessary_Net[i].Axis[1]);
			printf("Distance: %d\n",Necessary_Net[i].Distance);
			printf("Directions: ",i+1);
			puts(Necessary_Net[i].Direction+1);
			printf("\n");
		}
	}
	if(Obligatory_temp != 0){
		for(int i = 0;i<Obligatory_temp ; i++){
			printf("Obligatory Target[%d] Has Been Found !\n",i+1);
			printf("Axis: (%d, %d)  ",Obligatory_Net[i].Axis[0],Obligatory_Net[i].Axis[1]);
			printf("Distance: %d\n",Obligatory_Net[i].Distance);
			printf("Directions:  ",i+1);
			puts(Obligatory_Net[i].Direction+1);
			printf("\n");
		}
	}
}

void DFS(struct Map *p){
	int i = p->Axis[0]; //载入地图元素坐标 
	int j = p->Axis[1];
	Robot.Axis[0] = i; //更新机器人坐标至当前坐标 
	Robot.Axis[1] = j;
	
	Visit(p); //对当前节点进行访问 
	
	if(Check_NULL(p) == 0){
		Go_Back_FatherNet(p); //如果当前节点无子节点，则返回其父节点，递归返回，直到当前节点有子节点位置 
		if(p == &Map_Element[0][0]){ //如果返回到了原点，则表明深度优先搜索结束，程序退出 
			exit(0);
		}
	}else{ //如果有子节点的话则访问 
		for(int k = 0;k<8;k++){
		if(p->Next_Element[k]!=NULL) DFS(p->Next_Element[k]);
		}
	}	
}

int main(){
	Robot.Axis[0] = 0;
	Robot.Axis[1] = 0; //初始化机器人位置 
	Map_Save(); //从文件中读取地图 
	Map_Init(); //地图暂存数组初始化 
	
	DFS(Entrance); //深度优先搜索 
	
	return 0; 
}
