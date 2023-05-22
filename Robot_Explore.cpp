/*基础方位路径如下*/
/* 0 1 2
   3 p 4
   5 6 7*/  
/*如需改变地图尺寸则需修改宏定义中的row和column的值*/ 
/*如需改变程序运行速度则需改变如下程序中两个Sleep函数中参数的值*/
/*此程序并不支持入口点不在原点的情况*/

/*                         核心算法： 深度优先搜索                               */
/*-------------------------------------------------------------------------------*/
#include<stdio.h>
#include<stdlib.h>
#include<windows.h>
#include<string.h>


#define robot_position 'R'
#define row 18
#define column 56 //根据地图尺寸大小改变参数的值 
#define Max 100000

int Obligatory_temp = 0; //必须到达目标点个数统计 
int Necessary_temp = 0;//可到达的目标点 
int Target_temp = 0; //统计目标点个数下标 
int Target_number = 0; //目标点的个数 

char Direction_temp[10000] = {0}; //方位暂存数组 
int temp = 0;//方位暂存数组下标 
int finish = 0; //深度搜索结束标记 

int ID_T = 0; //后续函数中的全局变量 

char Map_temp[row][column];//地图元素暂存数组 
/*------------------------------------------------------------------------函数声明----------------------------------------------------------------------*/
void Map_Save();//保存地图 
void Map_Init();   //地图暂存数组初始化 
void Map_Element_Init(); //地图元素初始化 
void Map_show();  //画面输出1 
void Map_show2(); //画面输出2 
void Map_Show3(); //画面输出3 
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void Graph_Weight_Show(); //地图权重输出 
int Locate_CheckID(int ID, int ID2); //定位ID中的相邻目标点ID2在其结构体中的下标 
int Graph_Connect(int ID); //连接死点，叠加增权 
void Graph_Fix(); //修复邻接矩阵 
void Graph_Init(); //初始化图中元素 
void Simplify_Graph();  //回调死路，使问题有解 
int IF_Impasse(int ID); //检查是否为死节点 
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void Robot_Seen(); //机器人所能看到的范围 
void Check_Round_FindFather(struct Map *p,int flag); //检查附近的8个点是否可走，并连接 
int Check_NULL(struct Map *s); //检查是否为死节点 
void Visit(struct Map *p); //对目标节点进行访问 
void Go_Back_FatherNet(struct Map *p); //回到父节点 
int DFS(struct Map *p); //深度优先搜索【1】 
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void Reverse_Direction(char str[], char p[]); //将A->B的路径转化为B->A的路径 
void Visit2(struct Map *p); //对目标点进行访问 
void Go_Back_FatherNet2(struct Map *p); //回到父节点 
int DFS2(struct Map *p); //深度优先搜索【2】 
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void Adjacent_Target_Checker(); //搜索目标点的相邻目标点 
int DFS3(int ID); //深度优先搜索【3】 
void Back(int ID); //回到父节点 
void TSP(); //旅行商问题，贪心算法 
void Play_Shortest_Road(); //画面输出最短路径 
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
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

struct Target{ //所有目标点的结构体 
	int Axis[2];
	int ID;
	struct Target* Adjacent_Point[40]; //定义链表，连接目标点相邻的其他目标点 
	int Adjacent_Point_ID[40] = {0};  //相邻目标点的ID 
	int Adjacent_Point_Num = 0; //该目标点所拥有的相邻目标点的数量 
	char Direction[40][10000]; //该目标点到所有其相邻目标点的路径 
	int Distance[40]; //到所有相邻目标的距离 
}Target_Point[40];



struct Map{
	char nature; //该点的属性 
	int Axis[2]; //该点在图上的坐标 
	struct Map *Next_Element[8];//它周围可能的最多八个下一个节点 
	struct Map *Last_Element; //它的上一个节点 
	int Last_Element_Mark;
	int Invisited ; //是否被访问过 
	int ID;
	int Seen; //是否能被机器人看到 
}Map_Element[row][column],*Entrance = &Map_Element[0][0]; /*初始化地图元素结构体*/


/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

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
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
		Target_Point[Target_temp].ID = Target_temp;
		p->ID = Target_temp;
		Target_Point[Target_temp].Axis[0] = p->Axis[0]; //Target结构体同时记录 
		Target_Point[Target_temp].Axis[1] = p->Axis[1];
		Target_temp++; 
		Necessary_temp++;
	}
	if(p->nature == '#'){ //如果是Obligatory target 
		strcpy(Obligatory_Net[Obligatory_temp].Direction,Direction_temp);
		Obligatory_Net[Obligatory_temp].Axis[0] = p->Axis[0];
		Obligatory_Net[Obligatory_temp].Axis[1] = p->Axis[1];
		Obligatory_Net[Obligatory_temp].Distance = strlen(Direction_temp) - 1;
		p->ID = Target_temp;
		Target_Point[Target_temp].ID = Target_temp;
		Target_Point[Target_temp].Axis[0] = p->Axis[0];
		Target_Point[Target_temp].Axis[1] = p->Axis[1];
		Target_temp++; 
		Obligatory_temp++;
	}
	
	Map_Init();
	Map_temp[p->Axis[0]][p->Axis[1]] = robot_position;
	Robot.Axis[0] = p->Axis[0];
	Robot.Axis[1] = p->Axis[1];
	Sleep(100);
	system("cls");
	Map_show(); 
	/*初始化附地图暂存数组，地图清屏更新，呈现下一帧画面，机器人方位移动*/
}

void Visit2(struct Map *p){
	if(p->Axis[0]!=Target_Point[Target_temp].Axis[0]||p->Axis[1]!=Target_Point[Target_temp].Axis[1]){ 
		Direction_temp[temp] = p->Last_Element_Mark + '0'; //记录当前路径 
		Direction_temp[++temp] = '\0';
	}
	p->Invisited = 1;  //已访问过 
	Check_Round_FindFather(p,1); //检查该节点周围并寻找上一个节点 
	
	if((p->nature == '*'||p->nature == '#'||p == &Map_Element[0][0])&&(p->Axis[0]!=Target_Point[Target_temp].Axis[0]||p->Axis[1]!=Target_Point[Target_temp].Axis[1])){  //检查该目标点是不是自身 
		
		Target_Point[Target_temp].Adjacent_Point[Target_Point[Target_temp].Adjacent_Point_Num] = &Target_Point[p->ID];
		Target_Point[Target_temp].Distance[Target_Point[Target_temp].Adjacent_Point_Num] = strlen(Direction_temp);
		Target_Point[Target_temp].Adjacent_Point_ID[Target_Point[Target_temp].Adjacent_Point_Num] = p->ID;
		strcpy(Target_Point[Target_temp].Direction[Target_Point[Target_temp].Adjacent_Point_Num], Direction_temp);
		Target_Point[Target_temp].Adjacent_Point_Num++;
	}
	Map_Init();
	Map_temp[p->Axis[0]][p->Axis[1]] = robot_position;
	Robot.Axis[0] = p->Axis[0];
	Robot.Axis[1] = p->Axis[1];
	Sleep(100);
	system("cls");
	Map_show2(); 
	
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
			if(p != &Map_Element[0][0]){
				Map_temp[p->Last_Element->Axis[0]][p->Last_Element->Axis[1]] = robot_position;
				Robot.Axis[0] = p->Last_Element->Axis[0];
				Robot.Axis[1] = p->Last_Element->Axis[1];
			}else{
				Robot.Axis[0] = 0;
				Robot.Axis[1] = 0;
				Map_temp[0][0] = robot_position;
			}
			Sleep(100);
			system("cls");
			Map_show(); 
			
			if(p!=&Map_Element[0][0]){ 
				p->Last_Element->Next_Element[p->Last_Element_Mark] = NULL; //将前一个节点指向当前节点的指针设为空 
				p = p->Last_Element; //迭代到上一个节点 
			}else{
				flag = 1; //结束标记置1 
			}
		}
	}
	
}
void Go_Back_FatherNet2(struct Map *p){
	
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
			if(p->Axis[0]!=Target_Point[Target_temp].Axis[0]||p->Axis[1]!=Target_Point[Target_temp].Axis[1]){ //检查是不是自身 
				Map_temp[p->Last_Element->Axis[0]][p->Last_Element->Axis[1]] = robot_position;
				Robot.Axis[0] = p->Last_Element->Axis[0];
				Robot.Axis[1] = p->Last_Element->Axis[1];
			}else{
				Robot.Axis[0] = 0;
				Robot.Axis[1] = 0;
				Map_temp[0][0] = robot_position;
			}
			
			Sleep(100);
			system("cls");
			Map_show2(); 
			/*更新画面*/
			if(p->Axis[0]!=Target_Point[Target_temp].Axis[0]||p->Axis[1]!=Target_Point[Target_temp].Axis[1]){  //检查是不是自身 
				p->Last_Element->Next_Element[p->Last_Element_Mark] = NULL; //将前一个节点指向当前节点的指针设为空 
				p = p->Last_Element; //迭代到上一个节点 
			}else{
				flag = 1;
			} 
			
		}
	}
	
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void Map_Save(){
	/*载入地图*/
	FILE *fp; //文件指针 
	fp = fopen("Map1.txt","r"); //如需读取其他的地图需更改地图文件名称 
	char ch;
	int i = 0;
	int j = 0;
	while(fp != NULL&&!(i==row - 1&&j==column)){
		(Map_Element[i][j]).nature = fgetc(fp); //读一个指针后移一位 
		printf("%c",(Map_Element[i][j]).nature);
		Map_Element[i][j].Axis[0] = i; //存坐标 
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
	fclose(fp); //关文件 
}

void Map_Init(){
	for(int i = 0 ; i<row ; i++) for(int j=0 ;j < column ; j++) {
		Map_temp[i][j] = Map_Element[i][j].nature;
	}
} //从原结构体中读取 

void Map_Element_Init(){
	for(int i = 0; i < row; i++) for(int j = 0; j < column ; j++) Map_Element[i][j].Invisited = 0, Map_Element[i][j].Next_Element[8] = {NULL}, Map_Element[i][j].Last_Element_Mark = 0, Map_Element[i][j].Last_Element = NULL; //初始化 
}
void Robot_Seen(){
	int i = Robot.Axis[0];
	int j = Robot.Axis[1];
	//周围八个都是可以看到的，要控制边界 
	if(i-1>=0&&i-1<=row-1&&j-1>=0&&j-1<=column-1) Map_Element[i-1][j-1].Seen = 1;
	if(i-1>=0&&i-1<=row-1&&j>=0&&j<=column-1) Map_Element[i-1][j].Seen = 1;
	if(i-1>=0&&i-1<=row-1&&j+1>=0&&j+1<=column-1) Map_Element[i-1][j+1].Seen = 1;
	if(i>=0&&i<=row-1&&j-1>=0&&j-1<=column-1) Map_Element[i][j-1].Seen = 1;
	if(i>=0&&i<=row-1&&j+1>=0&&j+1<=column-1) Map_Element[i][j+1].Seen = 1;
	if(i+1>=0&&i+1<=row-1&&j-1>=0&&j-1<=column-1) Map_Element[i+1][j-1].Seen = 1;
	if(i+1>=0&&i+1<=row-1&&j>=0&&j<=column-1) Map_Element[i+1][j].Seen = 1;
	if(i+1>=0&&i+1<=row-1&&j+1>=0&&j+1<=column-1) Map_Element[i+1][j+1].Seen = 1;
	
}
/*--------------------------------------------------------------------Part 1----------------------------------------------------------------------------*/
void Map_show(){ /*输出当前地图*/
	Robot_Seen();
	for(int i = 0 ; i<row ; i++){
		for(int j = 0; j < column ; j++){
			if(finish == 0)	if(Map_Element[i][j].Seen == 1)printf("%c",Map_temp[i][j]);
							else printf(" ");
			else printf("%c",Map_temp[i][j]); 
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
			puts(Necessary_Net[i].Direction+1); //第一个元素不属于路径 
			printf("\n");
		}
	}
	if(Obligatory_temp != 0){
		for(int i = 0;i<Obligatory_temp ; i++){
			printf("Obligatory Target[%d] Has Been Found !\n",i+1);//从1开始 
			printf("Axis: (%d, %d)  ",Obligatory_Net[i].Axis[0],Obligatory_Net[i].Axis[1]);
			printf("Distance: %d\n",Obligatory_Net[i].Distance);
			printf("Directions:  ",i+1);
			puts(Obligatory_Net[i].Direction+1);
			printf("\n");
		}
	}
}

void Map_show2(){
	for(int i = 0 ; i<row ; i++){
		for(int j = 0; j < column ; j++){
			printf("%c",Map_temp[i][j]);
		}
		printf("\n");
	}
	printf("Please wait...... The robot is checking the targets' adjacent targets......\n\n");
	printf("Now, it is checking the Target_Point[%d]......\n\n", Target_temp);
	if(Target_Point[Target_temp].Adjacent_Point_Num != 0){
		printf("We have found its adjacent points : \n");
		for(int j = 0; j < Target_Point[Target_temp].Adjacent_Point_Num ; j++ ){
			printf("Adjacent Point[%d] : Target_Point[%d] \n", j,Target_Point[Target_temp].Adjacent_Point_ID[j]); //ID
			printf("Distance : %d  \nDirection : ", Target_Point[Target_temp].Distance[j]); //距离 
			puts(Target_Point[Target_temp].Direction[j]); //路径 
			printf("\n");
		}
	}
	
}

void Map_show3(){
	for(int i = 0 ; i<row ; i++){
		for(int j = 0; j < column ; j++){
			printf("%c",Map_temp[i][j]);
		}
		printf("\n");
	}
}

int DFS(struct Map *p){
	int i = p->Axis[0]; //载入地图元素坐标 
	int j = p->Axis[1];
	Robot.Axis[0] = i; //更新机器人坐标至当前坐标 
	Robot.Axis[1] = j;
	
	Visit(p); //对当前节点进行访问 
	
	if(Check_NULL(p) == 0){
		Go_Back_FatherNet(p); //如果当前节点无子节点，则返回其父节点，递归返回，直到当前节点有子节点位置 
		if(p == &Map_Element[0][0]){ //如果返回到了原点，则表明深度优先搜索结束，程序退出 
			finish = 1;
			return 0;
		}
		return 0;
	}else{ //如果有子节点的话则访问 
		for(int k = 7;k>=0;k--){
		if(p->Next_Element[k]!=NULL) DFS(p->Next_Element[k]);
		}
		return 0;
	}	
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------------*/
void Reverse_Direction(char str[], char p[])
{
    int i, j;
    char temp;
    int n = strlen(str);

    for (i = 0, j = n - 1; j >= 0; i++, j--) {
        *(p+i) = 7 - (str[j] - '0') + '0'; //跳转表，方位相对转换 
    }
    *(p+i) = '\0';
}

int DFS2(struct Map *s){
	Visit2(s); //对当前节点进行访问 
	if(Check_NULL(s) == 0){
		Go_Back_FatherNet2(s);
		return 0;
	}else{ //如果有子节点的话则访问 
		for(int k = 7;k>=0;k--){
			if(s->Next_Element[k]!=NULL)if(s->Next_Element[k]->nature == '*'||s->Next_Element[k]->nature == '#'){ //如果是目标点 
			 	 
			 	 for(int j = 0; j < 8 ; j++) if(j!=k){s->Next_Element[j] = NULL;  //断掉周围除目标点外的七个点，使其不能成为自己的下一个节点 
			 	  }
			 	  Visit2(s->Next_Element[k]); //访问目标点 
			 	 s = s->Next_Element[k]; //迭代 
			 	 for(int j = 0; j < 8 ; j++) s->Next_Element[j] = NULL; //断掉目标点周围的八个点，使其不能成为自己的下一个节点 
				 Go_Back_FatherNet2(s); //必须返回 
				 return 0;
			}
		}
		for(int k = 7; k>=0 ;k--){
			if(s->Next_Element[k]!=NULL) DFS2(s->Next_Element[k]); //如果周围八个点没有目标点则搜索正常进行 
		}
	}
	return 0;	
}

void Adjacent_Target_Checker(){
	struct Map *s;
	Direction_temp[10000] = {0}; //清空路径暂存数组 
	for(int i = 0; i< Target_number  ; i++){
		temp = 0;
		Direction_temp[10000] = {0};
		Map_Element_Init(); //初始化地图元素 
		s = &Map_Element[Target_Point[i].Axis[0]][Target_Point[i].Axis[1]]; //遍历所有目标点 
		s->Last_Element_Mark = 0; //初始点的上节点标记置为0 
		DFS2(s); //深度优先搜索 
		Target_temp++; //计数 
	}
	printf("Please hit enter to continue :\n");
	system("pause");
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------------*/
int Graph[40][40];
int Graph_Weight[40] = {0};
int DFS_Count = 0;
int Count = 0;
int Must = 0;

int Distance_temp[2]; //[0]储存当前距离， [1]储存最短距离 
int Invisited[40];
char Direction_Must[40][10000] = {0};
int Distance_Must[40] = {0};
int Direction_ID[2][40];//[0] 储存当前路径， [1]储存最短路径 
void Back(int ID);


int Locate_CheckID(int ID, int ID2){
	for(int i = 0 ;i < Target_Point[ID].Adjacent_Point_Num ; i++){
		if(Target_Point[ID].Adjacent_Point_ID[i] == ID2) return i; //搜索在目标结构体中ID2所对应的下标 
	}
	return -1; //如果搜索不到，就说明ID结构体中不存在ID2相邻目标点； 
}

int IF_Impasse(int ID){
	int count = 0;
	for(int i = 0; i < Target_number ; i++){
		if(Graph[ID][i]!=Max&&i!=ID&&Graph[ID][i] == Graph[i][ID]) count++;
	}
	if(count == 1) return 1;
	else return 0;
}

int Graph_Connect(int ID){
	if(IF_Impasse(ID)){//是死点 
			int Last_Net;
			int i = 0;
			for(i = 0; i < Target_number ; i++) if(i!=ID&&Graph[ID][i]!=Max) Last_Net = i;
			i = 0;
			char Direction_temp2[10000]; // 0000
			Reverse_Direction(Target_Point[Last_Net].Direction[Locate_CheckID(Last_Net, ID)],Direction_temp2); //1234 -> 6543 方位跳转，返回路径 
			strcat(Target_Point[Last_Net].Direction[Locate_CheckID(Last_Net, ID)], Direction_temp);  // 1234 + Direction 
			strcpy(Direction_temp,Target_Point[Last_Net].Direction[Locate_CheckID(Last_Net, ID)]);  // Direction = 1234 + Direction 
			strcat(Direction_temp, Direction_temp2);// Direction = 1234 + Direction + 6543 首尾相接  去了又回 
			Distance_temp[0] += 2*Graph[Last_Net][ID]; // + 往返距离 
			Graph[ID][Last_Net] = Max; //将两点的互通路径从图中抹去 
			Graph[Last_Net][ID] = Max;
			Graph_Weight[Last_Net] = Graph_Weight[ID] + 1; //权重迭代相加 
			Graph_Connect(Last_Net); //对死点的上一个节点进行调查，直到该节点不是死点 
	}else{ //不是死点 
		Distance_Must[ID] = Distance_temp[0]; //必经距离 
		strcpy(Direction_Must[ID], Direction_temp); //必经路径 
		Direction_temp[10000] = {0};
		Distance_temp[0] = 0;
	}
}

void Graph_Weight_Show(){ //输出图的权重 
	
	for(int i = 0; i < Target_number ; i++) printf("%7d", Graph_Weight[i]);printf("\n");
	printf("\n");
	printf("OK, We branch simplified the graph and these numbers are the weight coefficients of all target points in the graph\n\n");
	printf("Please hit enter to continue :\n");
	system("pause");
}

void Simplify_Graph(){ //简化图，增加权重，排除死点 

	for(ID_T= 1 ; ID_T < Target_number ; ID_T++){ //原点不能作为死点 
		Graph_Connect(ID_T);
		
	}
	Sleep(100);
	system("cls");
	Graph_Weight_Show();
}

void Graph_Fix(){ //排除图中两点间距离A->B 不等于 B->A的情况，对图进行修复 
	for(int i = 0; i < Target_number ; i++) for(int j = 0; j < Target_number ; j++){
		if(Graph[i][j] == Graph[j][i]){ //矩阵对称点相等则无需修复 
			continue;
		}else{ //否则 
			int shorter_id = (Graph[i][j] < Graph[j][i])? i : j; //取路径距离小的作为shorter 
			int longer_id = (Graph[i][j] < Graph[j][i])? j : i; //路径距离大的作为longer 
			int shorter_locate; //用于两点在互相的结构体中定位 
			int longer_locate;
			int check_have = 0; //可能存在对方在自己的结构体中不存在的情况，所以需要引入一个常量加以判断 
			for(int k = 0; k < Target_Point[shorter_id].Adjacent_Point_Num ; k++) {if(Target_Point[shorter_id].Adjacent_Point_ID[k] == j)  shorter_locate = k, check_have = 1; } //下标定位 
			if(check_have == 0) shorter_locate = ++Target_Point[shorter_id].Adjacent_Point_Num - 1, Target_Point[shorter_id].Adjacent_Point_ID[shorter_locate] = j; //如果不存在 
			check_have = 0;
			for(int k = 0; k < Target_Point[longer_id].Adjacent_Point_Num ; k++) {if(Target_Point[longer_id].Adjacent_Point_ID[k] == i)  longer_locate = k, check_have = 1;}
			if(check_have == 0) longer_locate = ++Target_Point[longer_id].Adjacent_Point_Num - 1, Target_Point[longer_id].Adjacent_Point_ID[longer_locate] = i; //强行引入 
			
			Target_Point[longer_id].Distance[longer_locate] = Target_Point[shorter_id].Distance[shorter_locate];  //Num++，路径和距离均统一为较短的一方 
			Reverse_Direction(Target_Point[shorter_id].Direction[shorter_locate],Target_Point[longer_id].Direction[longer_locate]); //对对方的路径进行逆向处理 
			
			Graph[longer_id][shorter_id] = Graph[shorter_id][longer_id]; //修改图 
		}
	}
}

void Graph_Init(){
	for(int i = 0; i < Target_number ; i++) for(int j = 0; j < Target_number ; j++){
		Graph[i][j] = Max;
		Graph_Weight[i] = 1; //初始化权重均为1 
	} 
	for(int i = 0; i < Target_number ; i++){
		Graph[i][i] = 0;
		for(int j = 0; j < Target_Point[i].Adjacent_Point_Num ; j++){
			Graph[i][Target_Point[i].Adjacent_Point_ID[j]] = Target_Point[i].Distance[j];
		}
	}
	Graph_Fix();
	system("cls");
	for(int i = 0; i < Target_number ; i++){ for(int j = 0; j < Target_number ; j++) printf("%7d",Graph[i][j]);printf("\n");}
	Sleep(100);
	printf("\n");
	printf("This is the adjacency matrix of all target points in the figure, where the number in row i and column j represents the distance between the target point i and the target point j. If the number is 100000, there is no direct path between the ith and the JTH target points, that is, they are not adjacent.\n\n");
	printf("Please hit enter to continue :\n");
	system("pause");
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int DFS3(int ID){
	 Count++; //计数器，作为数组下标 
	if(DFS_Count == Target_number - 1){ //如果权重已满 
	 	Distance_temp[0] += Graph[ID][0]; //将当前的路径距离加上该点到原点的路径距离 
	 	if(Distance_temp[1] == 0) Distance_temp[1] = Distance_temp[0]; //如果此时【1】的位置是0，则直接复制过去 
	 	if(Distance_temp[0] <= Distance_temp[1]){ //如果当前路径距离小于等于最短路径距离则更新 
	 		for(int i = 0 ; i < Target_number; i++) Direction_ID[1][i] = Direction_ID[0][i];
	 		Distance_temp[1] = Distance_temp[0];
	 		Back(ID); //返回 
	 		return 0; 
	 	}else{
	 		Back(ID); //否则直接返回 
	 		return 0;
	 	}
	}else{
		
		int i = 0;
		for( i = 0 ; i < Target_number; i++){
		 if(Graph[ID][i]!=Max&&Invisited[i]!=1&&i!=ID) {
		 	Invisited[i] = 1; //更新数据 
		 	Direction_ID[0][Count] = i;
			Distance_temp[0] += Graph[ID][i];
			DFS_Count += Graph_Weight[i]; //加权重 
			if(Graph_Weight[i]!=1){
				Distance_temp[0] += Distance_Must[ID];
			}
			DFS3(i) ;
		}
		}
		if(i == Target_number) Back(ID);
	}
}

void Back(int ID){
	DFS_Count -= Graph_Weight[ID]; //减权重 
	Count = Count - 1;
	Distance_temp[0] -= Graph[Direction_ID[0][Count-1]][ID];

	Direction_ID[0][Count] = 0;
	Invisited[ID] = 0;
}

void TSP(){ //贪心算法 
	Graph_Init(); //初始化图 
	Simplify_Graph(); //对图进行简化 
	for(int terminal = 1; terminal < Target_number ; terminal++){ //由于要完成原路返回的任务，不能将原点置为最终顶点。所以在这里分别取剩下的所有目标点分别作为最终顶点 
		if(Locate_CheckID(terminal,0)==-1) continue; //如果所取的最终顶点没有返回原点的路径，则进行下一次循环 
		Invisited[40] = {0}; //初始化 
		DFS_Count = 0; 
		Count = 0;
		Distance_temp[0] = 0;
		Direction_ID[0][0] = 0;
		Invisited[0] = 1;
		DFS3(0); //深度优先搜索 
	}		
		
}

void Play_Shortest_Road(){
	char Direction_Shortest[10000] = {0};
	int temp = 0;
	do{
		if(Graph_Weight[Direction_ID[1][temp]] != 1) strcat(Direction_Shortest, Direction_Must[Direction_ID[1][temp]]); //如果该点权重不为0，则需要连接必须经过的路径 
		if(temp == 0) strcpy(Direction_Shortest, Target_Point[Direction_ID[1][temp]].Direction[Locate_CheckID(Direction_ID[1][temp], Direction_ID[1][temp+1])]); //第一个点则字符串复制 
		else strcat(Direction_Shortest, Target_Point[Direction_ID[1][temp]].Direction[Locate_CheckID(Direction_ID[1][temp], Direction_ID[1][temp+1])]); //其余点连接 
		temp++;
	}while(Direction_ID[1][temp]!=0);
	int i = 0;
	int j = 0;
	temp = 0;
	int distance = 0;
	int num = 0;
	do{
		if(Direction_Shortest[temp] == '0') i--,j--; //路径转化为方位 
		if(Direction_Shortest[temp] == '1') i--;
		if(Direction_Shortest[temp] == '2') i--,j++;
		if(Direction_Shortest[temp] == '3') j--;
		if(Direction_Shortest[temp] == '4') j++;
		if(Direction_Shortest[temp] == '5') i++,j--;
		if(Direction_Shortest[temp] == '6') i++;
		if(Direction_Shortest[temp] == '7') i++,j++;
		/*if((Map_Element[i][j].nature == '*'||Map_Element[i][j].nature == '#')) num++;*/
		Map_Init();
		Map_temp[i][j] = robot_position;
		system("cls");
		Map_show3();
		printf("Please wait......\n");
		printf("The Robot is traveling in the most energy-efficient way possible ! \n\n");
		printf("Cumulative Distance : %d\n\n", distance);
		printf("Planned path : %s\n\n", Direction_Shortest);
		/*if(num!=0){
			for(int i = 1 ;i <= num; i++){
				printf("Arrive Target[%d]\n", Direction_ID[1][i]);
			}
		}*/
		Sleep(500);
		temp++;
		distance++;
	}while(temp <= strlen(Direction_Shortest)); 
	
	printf("Now, All the problems have been solved ! !\n\n");
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
int main(){
	Target_Point[0].Axis[0] = 0;
	Target_Point[0].Axis[1] = 0;
	Target_Point[0].ID = 0;
	Target_temp++;
	Robot.Axis[0] = 0;
	Robot.Axis[1] = 0; //初始化机器人位置 
	Map_Save(); //从文件中读取地图 
	Map_Init(); //地图暂存数组初始化 
	
	DFS(Entrance); //深度优先搜索 
	finish = 1;
	printf("\nPlease input 'Y' to continue \n");
	Target_number = Target_temp;
	Target_temp = 0;
	char Next_Order = '\0';
	Next_Order = getchar();
	
	if(Next_Order == 'Y'){
		temp = 0;
		Direction_temp[10000] = {0};
		
		system("cls");
		Map_Init();
		Map_Element_Init();
		Adjacent_Target_Checker();
		TSP();
		Distance_temp[0] = 0;
		Play_Shortest_Road();
	}
	system("pause");
	return 0; 
}
/*丁哥，你看我多爱你，七百多行代码我说写就写，你看看是不是，每个月那个钱，分我点*/
