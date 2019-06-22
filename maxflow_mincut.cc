#include<iostream>
#include<vector>
#include<queue>
#include<climits>

#define SOURCE_NODE_ID 0
#define TARGET_NODE_ID 5
#define NODE_COUNT 6
#define INVALID_PARENT -1

/*
 * Node id to name mapping
 *
 * s --> 0 
 * w --> 1 
 * x --> 2
 * z --> 3
 * y --> 4
 * t --> 5
 *
 */

static const char* nodeName[NODE_COUNT] = {"s", "w", "x", "z", "y", "t"};

using namespace std;

/* 
 * @brief    This class represents those nodes of the residual graphs that
 *          appear in the augmenting paths from source to target.
 *
 */
class residualGraphNode{
public:
    residualGraphNode(int id, int parent_id){
        id_ = id;
        parent_id_ = parent_id;
    }
    int getId(){return id_;}
    void setId(int id){id_ = id;}
    int getParentId(){return parent_id_;}
    void setParentId(int parent_id){parent_id_ = parent_id;}
private:
    int id_; /* Id of node */
    int parent_id_; /* Id of parent node in an augmenting path */
};

/* 
 * @breif    This class represents edges in a residual graph.
 *
 */
class residualGraphEdge{
public:
    residualGraphEdge(int residual_capacity, int original_capacity){
        residual_capacity_ = residual_capacity;
        original_capacity_ = original_capacity;
    }
    int getResidualCapacity(){return residual_capacity_;}
    void setResidualCapacity(int residual_capacity){residual_capacity_ = residual_capacity;}
    int getOriginalCapacity(){return original_capacity_;}
    void setOriginalCapacity(int original_capacity){original_capacity_ = original_capacity;}
private:
    int residual_capacity_; /* residual capacity of an edge */
    int original_capacity_; /*  Original capacity of an edge */
};

/*
 *  @brief  BFS implementation that runs on a graph to find out a path between
 *          the source node and the target node.
 *
 *  @param[in]             graph    The graph to traverse.
 *  @param[out] (optional) sPath    The nodes reachable from the source after bfs is done. The default value is NULL.
 *  @param[out] (optional) tPath    The nodes not reachable from the source after bfs is done. The default value is NULL.
 *  @param[out] (optional) outPath  Path from source to target. The default value is NULL.
 *  
 *  @return True if the target node is reachable from the source
 *
 */
bool bfs(vector<vector<residualGraphEdge> > &graph,
        vector<int>* sPath = NULL,
        vector<int>* tPath = NULL,
        vector<residualGraphNode> *outPath = NULL)
{

    vector<bool> trav; /* For each node in the graph, we will store true if we can reach from the source */
    for(int i = 0; i< NODE_COUNT; i++){
        /* Initialize with false value for each node */
        trav.push_back(false);
    }
    
    queue<int> bfs_q;
    bfs_q.push(SOURCE_NODE_ID); /* Start with the cource */
    trav[SOURCE_NODE_ID] = true;
    while(!bfs_q.empty()){
        int node_id = bfs_q.front();
        bfs_q.pop();
        for(int next_node = 0; next_node < NODE_COUNT; next_node++){
            if(!trav.at(next_node) && graph[node_id][next_node].getResidualCapacity() > 0){
                /* 
                 * This is an unvisited node and the edge it shares with the previous node
                 * has some residual capacity value. So Mark it traversed and mark the previous
                 * node as it's parent. 
                 */
                trav[next_node] = true;
                if(outPath != NULL) {
                    outPath->at(next_node).setParentId(node_id);
                }
                bfs_q.push(next_node);
            }
        }

    }
    /* 
     * This part is used to compute the s-t cut.
     * Now that we know, which nodes can be reached 
     * from the source and which nodes can not be,
     * store them in the respective buffers. Use this
     * code when there is no augmenting path left.
     */
    if (sPath != NULL && tPath != NULL) {

        for(int i = 0; i< NODE_COUNT; i++){
            if(trav.at(i)){
                sPath->push_back(i);
            } else {
                tPath->push_back(i);
            }
        }
    }

    return trav.at(TARGET_NODE_ID);
}
/*
 * @brief   Implementation of the Ford Fulkerson algorithm to compute
 *          the maximum flow possible in a flow network
 *
 * @param [in]  residualGraph   A flow network transformed into a residual graph
 *                              with back edges
 *
 * @return  Returns the maximum flow possible in that flow network
 */
int fordFulkerson(
        vector<vector<residualGraphEdge> > &residualGraph)

{
    vector<residualGraphNode> augmentingPath; /* We call BFS and store the augmenting path at every stage*/
    int max_flow = 0;

    for(int i = 0; i< NODE_COUNT; i++){
        /* Initialize the path */
        augmentingPath.push_back(residualGraphNode(i, INVALID_PARENT));
    }


    while(bfs(residualGraph, NULL, NULL, &augmentingPath)){
        /* An augmenting path was found */
        int min_flow_in_path = INT_MAX;

        /* 
         * Compute the minimum capacity value in this augmenting path.
         * This should be the maximum amount of flow possible using
         * this path.
         */
        for(int node_id = TARGET_NODE_ID; node_id != SOURCE_NODE_ID;){
            int parent_node_id = augmentingPath[node_id].getParentId();
            if(min_flow_in_path> residualGraph[parent_node_id][node_id].getResidualCapacity()) {
                min_flow_in_path = residualGraph[parent_node_id][node_id].getResidualCapacity();
            }
            node_id = parent_node_id;
        }
        /* Update the max_flow value for this path. */
        max_flow += min_flow_in_path;

        for(int node_id = TARGET_NODE_ID; node_id != SOURCE_NODE_ID;){
            int parent_node_id = augmentingPath[node_id].getParentId();
            augmentingPath[node_id].setParentId(INVALID_PARENT);
            /* 
             * Update the edges and the back edges with the max poossible flow found for this
             * path. So we add the max flow value for this path to the back edge capacities
             * and subtract the value from the edges.
             */
            residualGraph[parent_node_id][node_id].setResidualCapacity(
                    residualGraph[parent_node_id][node_id].getResidualCapacity() - min_flow_in_path
                    );
            residualGraph[node_id][parent_node_id].setResidualCapacity(
                    residualGraph[node_id][parent_node_id].getResidualCapacity() + min_flow_in_path
                    );
            node_id = parent_node_id;
        }
    }

    /*
     * Now print the flows for all the edges that sum up to the maximum flow.
     * The difference between the residual flow and the original flow should be answer
     * for each edge.
     */

    cout<< "Printing flows through all the edges that sum up to the maximum flow.\n";
    cout<< "Flow through s->w: "<<residualGraph[0][1].getOriginalCapacity()-residualGraph[0][1].getResidualCapacity()<<"\n";
    cout<< "Flow through s->x: "<<residualGraph[0][2].getOriginalCapacity()-residualGraph[0][2].getResidualCapacity()<<"\n";
    cout<< "Flow through s->z: "<<residualGraph[0][3].getOriginalCapacity()-residualGraph[0][3].getResidualCapacity()<<"\n";
    cout<< "Flow through w->y: "<<residualGraph[1][4].getOriginalCapacity()-residualGraph[1][4].getResidualCapacity()<<"\n";
    cout<< "Flow through w->t: "<<residualGraph[1][5].getOriginalCapacity()-residualGraph[1][5].getResidualCapacity()<<"\n";
    cout<< "Flow through x->w: "<<residualGraph[2][1].getOriginalCapacity()-residualGraph[2][1].getResidualCapacity()<<"\n";
    cout<< "Flow through x->z: "<<residualGraph[2][3].getOriginalCapacity()-residualGraph[2][3].getResidualCapacity()<<"\n";
    cout<< "Flow through x->y: "<<residualGraph[2][4].getOriginalCapacity()-residualGraph[2][4].getResidualCapacity()<<"\n";
    cout<< "Flow through z->y: "<<residualGraph[3][4].getOriginalCapacity()-residualGraph[3][4].getResidualCapacity()<<"\n";
    cout<< "Flow through z->t: "<<residualGraph[3][5].getOriginalCapacity()-residualGraph[3][5].getResidualCapacity()<<"\n";
    cout<< "Flow through y->w: "<<residualGraph[4][5].getOriginalCapacity()-residualGraph[4][5].getResidualCapacity()<<"\n";

    return max_flow;
}

/*
 * @brief   Find the minimum s-t cut of the input residual graph. Print the s and t vertices sets 
 *          on the standard output console.
 * @param[in] graph A residual graph
 *
 */

void findMinCut(vector<vector<residualGraphEdge> >& graph){

    vector<int> sPath;
    vector<int> tPath;
    if (bfs(graph, &sPath, &tPath)){
        cout<<"The residual graph still has one or more augmenting paths. Failed to compute minimum s-t cut.\n";
        return;
    }
    cout<<"\nMinimum s-t cut \n"; 
    cout<<"Nodes at the s side:\n";
    for(vector<int>::iterator itr = sPath.begin(); itr!= sPath.end(); itr++){
        cout <<nodeName[*itr]<<" ";
    }
    cout<<"\n";

    cout<<"Nodes at the t side:\n";
    for(vector<int>::iterator itr = tPath.begin(); itr != tPath.end(); itr++){
        cout <<nodeName[*itr]<<" ";
    }
    cout<<"\n";
}

/*
 * @brief   Main function to call the Ford Fulkerson and MinCut
 *          functions. The input graph is hard coded. So it doesn't
 *          take any other input.
 *
 * @return Returns 0 on success
 */
int main(){
    vector<vector<residualGraphEdge> > graph; /* Container to store the input graph */

    /* 
     * Create the input graph as a residual graph. We add back edges with 0 capacity
     * to begin with and update them later.
     */
    for(int i = 0; i< NODE_COUNT; i++){
        vector<residualGraphEdge> edge_set_to_node;
        for(int i = 0; i< NODE_COUNT; i++){
            /* Initialize all the edges with 0 capacity */
            edge_set_to_node.push_back(residualGraphEdge(0,0));
        }
        graph.push_back(edge_set_to_node);
    }

    /* 
     * Update actual graph edges with their residual capacities.
     * Initially it is same as the original capacity.
     */
    graph[0][1].setResidualCapacity(4); /* s->w */
    graph[0][2].setResidualCapacity(7); /* s->x */
    graph[0][3].setResidualCapacity(10); /* s->z */
    graph[1][4].setResidualCapacity(2); /* w->y */
    graph[1][5].setResidualCapacity(10); /* w->t */
    graph[2][1].setResidualCapacity(2); /* x->w */
    graph[2][3].setResidualCapacity(2); /* x->z */
    graph[2][4].setResidualCapacity(10); /* x->y */
    graph[3][4].setResidualCapacity(2); /* z->y */
    graph[3][5].setResidualCapacity(6); /* z->t */
    graph[4][5].setResidualCapacity(7); /* y->t */

    /* Now update the original capacity of the edges */
   
    graph[0][1].setOriginalCapacity(4); /* s->w */
    graph[0][2].setOriginalCapacity(7); /* s->x */
    graph[0][3].setOriginalCapacity(10); /* s->z */
    graph[1][4].setOriginalCapacity(2); /* w->y */
    graph[1][5].setOriginalCapacity(10); /* w->t */
    graph[2][1].setOriginalCapacity(2); /* x->w */
    graph[2][3].setOriginalCapacity(2); /* x->z */
    graph[2][4].setOriginalCapacity(10); /* x->y */
    graph[3][4].setOriginalCapacity(2); /* z->y */
    graph[3][5].setOriginalCapacity(6); /* z->t */
    graph[4][5].setOriginalCapacity(7); /* y->t */
	
	/* There are no self loops, so we do not need to modify this graph */

    cout<<"Max flow found after running Ford Fulkerson algorithm: "<<fordFulkerson(graph)<<"\n";

    /* 
     * Now that the Ford Fulkerson algorithm has been executed, our residual graph does not have
     * any augmenting path. Now use this graph to get the s-t cut vertices sets.
     */

    findMinCut(graph);
    
    return 0;
}
