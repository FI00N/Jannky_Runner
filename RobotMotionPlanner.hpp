#include <string.h>
#include "SimpleQueue.hpp"

namespace mtrn3100 {

class RobotMotionPlanner {
private:
    struct Node {
        int vertex;
        Node* next;
    };

    Node* adjList[MAX_SIZE];
    bool visited[MAX_SIZE];
    int prevNode[MAX_SIZE];

public:
    
    enum Direction {N, S, W, E};
    
    RobotMotionPlanner() {
        for (int i = 0; i < MAX_SIZE; i++) {
            adjList[i] = nullptr;
            visited[i] = false;
            prevNode[i] = -1;
        }
    }

    ~RobotMotionPlanner() {
        for (int i = 0; i < MAX_SIZE; i++) {
            while (adjList[i]) {
                Node* tmp = adjList[i];
                adjList[i] = adjList[i]->next;
                delete tmp;
            }
        }
    }

    Direction getStartDirection(char str[]) {
        return parseDirection(str[strlen(str) - 1]);
    }
    
    Direction parseDirection(char dirChar) {
        switch (dirChar) {
            case 'N': return N;
            case 'S': return S;
            case 'W': return W;
            case 'E': return E;
            default: return N;
        }
    }

    void getMotion(Direction currentDir, Direction targetDir, char* motion) {
        if (currentDir == targetDir) {
            strcpy(motion, "F");
            return;
        }
        if ((currentDir == N && targetDir == W) ||
            (currentDir == W && targetDir == S) ||
            (currentDir == S && targetDir == E) ||
            (currentDir == E && targetDir == N)) {
            strcpy(motion, "LF");
            return;
        }
        if ((currentDir == N && targetDir == E) ||
            (currentDir == E && targetDir == S) ||
            (currentDir == S && targetDir == W) ||
            (currentDir == W && targetDir == N)) {
            strcpy(motion, "RF");
            return;
        }
        strcpy(motion, "RRF");
    }

    Direction getDirection(int current, int next) {
        if (next == current + 1) return E;
        if (next == current - 1) return W;
        if (next == current + COL) return S;
        if (next == current - COL) return N;
        return N;
    }

    void generateMotionPlan(const int* path, int pathSize, Direction start_pose, char* motionPlan) {
        if (pathSize < 2) {
            strcpy(motionPlan, "");
            return;
        }

        Direction currentDir = S;
        
        if (start_pose == S) {
            strcpy(motionPlan, "");
        }
        if (start_pose == N) {
            strcpy(motionPlan, "RR");
        }
        if (start_pose == W) {
            strcpy(motionPlan, "L");
        }
        if (start_pose == E) {
            strcpy(motionPlan, "R");
        }

        char motion[4];
        for (int i = 0; i < pathSize - 1; ++i) {
            Direction targetDir = getDirection(path[i], path[i + 1]);
            getMotion(currentDir, targetDir, motion);
            strcat(motionPlan, motion);
            currentDir = targetDir;
        }
    }

    void addEdge(int src, int dst) {
        Node* newNode = new Node{dst, nullptr};
        if (!adjList[src]) {
            adjList[src] = newNode;
        } else {
            Node* current = adjList[src];
            while (current->next) {
                current = current->next;
            }
            current->next = newNode;
        }
    }
    
    int bfs(char startStr[], int end, int* resultPath) {
        int start = 0;
        for (int i = 0; startStr[i] >= '0' && startStr[i] <= '9'; i++) {
            start = start * 10 + (startStr[i] - '0');
        }
        SimpleQueue q;
        q.push(start);
        visited[start] = true;
        int pathSize = 0;

        while (!q.empty()) {
            int node = q.front();
            q.pop();

            if (node == end) {
                for (int at = end; at != -1; at = prevNode[at]) {
                    resultPath[pathSize++] = at;
                }
                
                // Reversing the resultPath
                int startIdx = 0, endIdx = pathSize - 1;
                while (startIdx < endIdx) {
                    int temp = resultPath[startIdx];
                    resultPath[startIdx] = resultPath[endIdx];
                    resultPath[endIdx] = temp;

                    startIdx++;
                    endIdx--;
                }
                
                return pathSize;
            }
            
            Node* current = adjList[node];
            while (current) {
                int neighbor = current->vertex;
                if (!visited[neighbor]) {
                    q.push(neighbor);
                    visited[neighbor] = true;
                    prevNode[neighbor] = node;
                }
                current = current->next;
            }
        }
        return 0;
    }
    
};

}  // namespace mtrn3100
