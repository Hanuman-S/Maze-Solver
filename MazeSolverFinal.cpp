#include <bits/stdc++.h>
using namespace std;
using namespace std::chrono;
// ANSI color codes
#define RESET   "\033[0m"
#define BWHITE  "\033[47m"  // White background
#define BBLACK  "\033[40m"  // Black background
#define BBLUE   "\033[44m"  // Blue background
#define BRED    "\033[41m"  // Red background

void printColoredMaze(vector<vector<int>>& maze) {
    cout << "\nMaze Visualization:\n";
    for (auto &row : maze) {
        for (int cell : row) {
            if (cell == 1) {                 // Wall
                cout << BBLACK << "  " << RESET;
            }
            else if (cell == 0) {            // Unvisited
                cout << BWHITE << "  " << RESET;
            }
            else if (cell == 2) {            // Visited
                cout << BBLUE << "  " << RESET;
            }
            else if (cell == 3) {            // Final path
                cout << BRED << "  " << RESET;
            }
        }
        cout << endl;
    }
}


struct Cell { int x, y; };

bool isValid(int x, int y, vector<vector<int>>& maze, vector<vector<bool>>& visited) {
    int n = maze.size(), m = maze[0].size();
    return (x>=0 && x<n && y>=0 && y<m && maze[x][y]==0 && !visited[x][y]);
}

void printMaze(vector<vector<int>>& maze) {
    cout << "\nFinal Maze State:\n";
    for(auto &r : maze) {
        for(int c : r) cout << c << " ";
        cout << "\n";
    }
}

void markPath(vector<vector<int>>& maze, vector<Cell>& path) {
    for(auto &p : path) maze[p.x][p.y] = 3;
}

void printPathAndMark(vector<vector<Cell>>& parent, Cell goal, vector<vector<int>>& maze) {
    vector<Cell> path;
    Cell cur = goal;

    while(parent[cur.x][cur.y].x != -1) {
        path.push_back(cur);
        cur = parent[cur.x][cur.y];
    }
    path.push_back(cur);
    reverse(path.begin(), path.end());

    cout << "\nPath: ";
    for(auto &p : path) cout << "(" << p.x << "," << p.y << ") ";
    cout << "\nPath length: " << path.size()-1 << "\n";

    markPath(maze, path);
}

//////////////////// BFS ////////////////////
void BFS(vector<vector<int>> maze, Cell start, Cell goal) {

    int n=maze.size(), m=maze[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m,false));
    vector<vector<Cell>> parent(n, vector<Cell>(m, {-1,-1}));

    queue<Cell> q; q.push(start); visited[start.x][start.y] = true;
    int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};

    while(!q.empty()) {
        Cell cur = q.front(); q.pop();
        if(!(cur.x == start.x && cur.y == start.y)) maze[cur.x][cur.y] = 2;

        if(cur.x == goal.x && cur.y == goal.y) {
            cout << "\nBFS: Shortest path found\n";
            printPathAndMark(parent, goal, maze);
            printColoredMaze(maze);
            break;
        }

        for(int k=0;k<4;k++){
            int nx=cur.x+dx[k], ny=cur.y+dy[k];
            if(isValid(nx,ny,maze,visited)) {
                visited[nx][ny]=true;
                parent[nx][ny]=cur;
                q.push({nx,ny});
            }
        }
    }
}

//////////////////// DFS ////////////////////
bool DFSRec(int x,int y,Cell goal, vector<vector<int>>& maze, vector<vector<bool>>& visited, vector<Cell>& path) {
    visited[x][y]=true;
    path.push_back({x,y});
    maze[x][y]=2;

    if(x==goal.x && y==goal.y) return true;

    int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};
    for(int k=0;k<4;k++){
        int nx=x+dx[k], ny=y+dy[k];
        if(isValid(nx,ny,maze,visited)) {
            if(DFSRec(nx,ny,goal,maze,visited,path)) return true;
        }
    }
    path.pop_back();
    return false;
}

void DFS(vector<vector<int>> maze, Cell start, Cell goal){

    int n=maze.size(),m=maze[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m,false));
    vector<Cell> path;

    if(DFSRec(start.x,start.y,goal,maze,visited,path)){
        cout << "\nDFS: Path found (not always shortest)\n";
        markPath(maze,path);
        cout << "Path length: " << path.size()-1 << "\n";
        printColoredMaze(maze);
    } else cout << "\nNo Path!\n";
}

//////////////////// Dijkstra ////////////////////
struct NodeD { int x,y,d; };
struct CompareD { bool operator()(NodeD const&a, NodeD const&b){ return a.d>b.d; } };

void Dijkstra(vector<vector<int>> maze, Cell start, Cell goal){

    int n=maze.size(),m=maze[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m,false));
    vector<vector<int>> dist(n, vector<int>(m,INT_MAX));
    vector<vector<Cell>> parent(n, vector<Cell>(m, {-1,-1}));

    priority_queue<NodeD,vector<NodeD>,CompareD> pq;
    pq.push({start.x,start.y,0}); dist[start.x][start.y]=0;

    int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};

    while(!pq.empty()){
        auto cur=pq.top(); pq.pop();
        if(visited[cur.x][cur.y]) continue;
        visited[cur.x][cur.y]=true;
        if(!(cur.x==start.x && cur.y==start.y)) maze[cur.x][cur.y] = 2;

        if(cur.x==goal.x && cur.y==goal.y){
            cout << "\nDijkstra: Shortest path found\n";
            printPathAndMark(parent,goal,maze);
            printColoredMaze(maze);
            break;
        }

        for(int k=0;k<4;k++){
            int nx=cur.x+dx[k], ny=cur.y+dy[k];
            if(isValid(nx,ny,maze,visited)){
                int nd=dist[cur.x][cur.y]+1;
                if(nd<dist[nx][ny]){
                    dist[nx][ny]=nd;
                    parent[nx][ny]={cur.x,cur.y};
                    pq.push({nx,ny,nd});
                }
            }
        }
    }
}

//////////////////// A* ////////////////////
struct NodeA { int x,y,g,h,f; };
struct CompareA { bool operator()(NodeA const&a, NodeA const&b){ return a.f>b.f; } };

int heuristic(Cell a, Cell b){ return abs(a.x-b.x)+abs(a.y-b.y); }

void Astar(vector<vector<int>> maze, Cell start, Cell goal){

    int n=maze.size(),m=maze[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m,false));
    vector<vector<Cell>> parent(n, vector<Cell>(m,{-1,-1}));

    priority_queue<NodeA,vector<NodeA>,CompareA> pq;
    pq.push({start.x,start.y,0,heuristic(start,goal),heuristic(start,goal)});
    visited[start.x][start.y]=true;

    int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};

    while(!pq.empty()){
        auto cur=pq.top(); pq.pop();
        if(!(cur.x==start.x && cur.y==start.y)) maze[cur.x][cur.y]=2;

        if(cur.x==goal.x && cur.y==goal.y){
            cout << "\nA*: Fastest optimal path method\n";
            printPathAndMark(parent,goal,maze);
            printColoredMaze(maze);
            break;
        }

        for(int k=0;k<4;k++){
            int nx=cur.x+dx[k], ny=cur.y+dy[k];
            if(isValid(nx,ny,maze,visited)){
                visited[nx][ny]=true;
                parent[nx][ny]={cur.x,cur.y};
                int g=cur.g+1, h=heuristic({nx,ny},goal), f=g+h;
                pq.push({nx,ny,g,h,f});
            }
        }
    }
}

//////////////////// main ////////////////////
int main(){
    vector<vector<int>> maze = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,1,0,1,0,0,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,1},
{1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1},
{1,0,1,1,1,1,1,0,1,0,0,1,0,1,1,0,0,0,1,1,0,0,1,0,1,0,1,0,1,0,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
{1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,1},
{1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,0,0,0,1,0,1,1,0,0,1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,0,1},
{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1},
{1,0,1,1,1,1,1,0,1,0,1,0,0,0,0,1,1,0,1,1,1,1,0,0,1,0,1,0,1,0,1,1,1,0,0,1,1,0,1,0,1,0,1,0,0,1,1,1,0,0,1},
{1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,1},
{1,0,1,1,0,0,1,0,0,0,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,0,1,1},
{1,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1},
{1,1,1,1,1,0,0,1,1,0,1,0,1,0,1,0,1,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1,1,0,1,0,1,0,1,0,1,1,0,1,1,1,1,1,1,0,1},
{1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1},
{1,0,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1},
{1,0,1,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,1,1,1,0,1,0,1,0,0,0,0,1,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1},
{1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1},
{1,0,1,1,1,1,1,0,0,1,1,1,1,0,1,0,1,0,1,1,1,0,0,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1,1,1,0,1,0,1,1,0,0,1},
{1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1},
{1,1,1,1,1,1,1,0,1,0,1,0,1,1,1,0,1,1,1,1,0,1,1,0,1,0,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,1,1,1,1,1,1,1,1,0,1},
{1,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1},
{1,0,1,1,1,0,0,0,1,0,1,0,0,1,1,0,1,1,1,1,0,0,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,0,1,1,0,1,0,1,1,1,0,0,0,1},
{1,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1},
{1,0,1,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,1,1,0,1,0,1,0,0,0,0,0,1,0,1,1,1,1,1,0,1,1,1,0,1},
{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1},
{1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,1,1,0,0,1,1,0,1,1,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1},
{1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1},
{1,0,1,1,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1,1,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,1},
{1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1},
{1,0,0,0,1,1,0,0,1,0,1,1,1,0,1,1,1,1,1,0,1,1,0,1,1,0,0,0,1,1,1,1,1,1,1,0,0,1,1,0,1,1,1,0,1,0,1,0,1,0,1},
{1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1},
{1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,0,0,0,1,1,0,0,0,0,1,0,0,1,0,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1},
{1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1},
{1,0,1,1,1,0,1,1,1,0,1,0,0,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1},
{1,0,1,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,1,0,0,0,0,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1,0,1,0,1},
{1,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,1},
{1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,1,1,0,0,1,1,1,1,0,0,0,1,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,1,1,1,0,1,0,1,0,1},
{1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,1},
{1,0,1,1,1,0,1,0,1,1,1,1,0,0,1,0,1,1,0,0,1,1,1,0,1,1,1,0,1,1,0,1,1,0,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1},
{1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,1,0,1,1,1,0,1,1,1,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1,1,0,1},
{1,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1},
{1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,0,0,1,1,1,0,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1},
{1,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1},
{1,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,0,1,1,1,0,1,1,1,0,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1},
{1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1},
{1,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1},
{1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    };

    Cell start={1,1}, goal={49,49};

    DFS(maze,start,goal);
    BFS(maze,start,goal);
    Dijkstra(maze,start,goal);
    Astar(maze,start,goal);
    
    return 0;
}
