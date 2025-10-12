//Problem statement:
//Given an undirected or directed graph represented as an adjacency list, perform a Depth First Search starting from vertex 0 and return the order of traversal.

//recursive approach
//Initialization:
//Create a boolean[] visited array of size V to track visited nodes.create an empty ArrayList<Integer> result to store the DFS traversal order.

//recursive DFS Function:
//mark the current node as visited. Add the current node to the result.

//Iterate over each neighbor of the current node from the adjacency list.
//If the neighbor is not visited, recursively call DFS on it.

//time complexity:
//Each vertex is visited once: O(V)
//each edge is explored once in directed graph so, O(E)
//Total: O(V + E)

//Space Complexity
//Visited array=O(V)
//Recursion stack space (in worst case, graph is a chain)=O(V)
//Result list=O(V)
//total Auxiliary Space is O(V)(not counting adjacency list storage)

//Code

import java.util.ArrayList;

public class Dfs {
    // Function to return a list containing the DFS traversal of the graph.
    
    public ArrayList<Integer> dfs(ArrayList<ArrayList<Integer>> adj) {
        // Code here
        int V=adj.size();
        boolean[] vis=new boolean[V];
        //dfs order
        ArrayList<Integer> res=new ArrayList<>();
        depthFirstSearch(0,adj,vis,res);
        return res;
    } 
    void depthFirstSearch(int node,ArrayList<ArrayList<Integer>> adj,boolean[] visited,ArrayList<Integer> result){
        visited[node]=true;
        result.add(node);//add it to the dfs
        for(int neighbor: adj.get(node)){
            if(!visited[neighbor])//check if that node is already visited or not
            {
                depthFirstSearch(neighbor,adj,visited,result);//if not visited explore that node
            }
        }
    }
    
    //Iterative approach
    //create a visited array to track visited nodes. 
    //Create a stack and push the starting node and mark it as visited.
    //while stack is not empty:
        //Pop the top node. add it to the result list.
        //for each unvisited neighbor of the popped node-
        //if it's not visited:
            //Mark it as visited.
            //Push all its unvisited neighbors onto the stack.
    //Repeat until the stack is empty.


//     public ArrayList<Integer> dfs(ArrayList<ArrayList<Integer>> adj) {
//         int V=adj.size();//number of vertices/nodes
//         //create a visited array
//         boolean[] vis=new boolean[V];
        
//         ArrayList<Integer> res=new ArrayList<>();
        
//         //take a stack and push the starting node onto it
//         Stack<Integer> st=new Stack<>();
//         st.push(0);//as start node is 0
//         vis[0]=true;
        
//         //process the nodes
//         while(!st.isEmpty()){
//             int node=st.pop();
//             res.add(node);//after poping add it to the dfs order list
            
//             //Push the neighbors onto the stack in reverse order to get the same result as the recursive approach
//             List<Integer> neighbors = adj.get(node);
//             for (int i = neighbors.size() - 1; i >= 0; i--) {
//                 int neighbor = neighbors.get(i);
//                 if (!vis[neighbor]) {
//                     vis[neighbor]=true;
//                     st.push(neighbor);
//                 }
//             }
// }
// return res;
// }

public static void main(String[] args) {
        
        ArrayList<ArrayList<Integer>> adj = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            adj.add(new ArrayList<>());
        }
        adj.get(0).add(1);
        adj.get(0).add(2);
        adj.get(1).add(3);
        adj.get(1).add(4);
        adj.get(2).add(4);
        
        Dfs d=new Dfs();
        ArrayList<Integer> result = d.dfs(adj);
        System.out.println(result); 
    }
}
//output:[0, 1, 3, 4, 2]