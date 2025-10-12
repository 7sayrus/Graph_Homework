//problem statement:
//Given a graph represented as an adjacency list, perform a Breadth-First Search (BFS) traversal starting from vertex 0.

//approach:
//take a boolean array vis[] of size vertices to mark visited vertices.
//take a Queue<Integer> to store vertices for BFS processing.
//take an result ArrayList<Integer> to store the BFS traversal order.

//Start from vertex 0
//mark it as visited and add it to the queue.
//to process the queue-
//While the queue is not empty--
//pop the front element and add it to the bfs result list.
//For each neighbor of the popped node--
//check if it is visited or not if not visited then mark it as visited and add it to the queue.
//return the BFS traversal list.

//time Complexity:
//each vertex is enqueued and dequeued once so, O(V)
//each edge is checked at most twice (as undirected graph) so O(E)
//so, O(V + E)

//Space Complexity:
//for vis[] array  O(V) and result list -> O(V)
//So, O(V)

//code:
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class Bfs{
  public ArrayList<Integer> bfs(ArrayList<ArrayList<Integer>> adj) {
        
        int vertices=adj.size();
        //visited array
        boolean[] vis=new boolean[vertices];
        
        //result list that contains the nodes after bfs traversal
        ArrayList<Integer> res=new ArrayList<>();
        
        //queue
        Queue<Integer> q=new LinkedList<>();
        q.add(0);
        vis[0]=true;
        while(!q.isEmpty()){
              Integer node=q.poll();
              res.add(node);
              for(Integer neighbor: adj.get(node)){
                  if(vis[neighbor]==false){
                      vis[neighbor]=true;
                      q.add(neighbor);
                  }
            }
        }
      return res;  
    }
    public static void main(String args[]) {
        Bfs b = new Bfs();
        ArrayList<ArrayList<Integer>> adj = new ArrayList<>();
        
        //adjacency list
        for (int i = 0; i < 5; i++) {
            adj.add(new ArrayList<>());
        }
        adj.get(0).add(1);
        adj.get(0).add(2);
        adj.get(1).add(3);
        adj.get(1).add(4);
        
        ArrayList<Integer> result = b.bfs(adj);
        System.out.println(result); 

    }
}

// Output: [0, 1, 2, 3, 4]