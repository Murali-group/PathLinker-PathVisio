package pathlinker;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Queue;
import java.util.Scanner;
import javax.swing.JOptionPane;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.DefaultGraph;
import org.pathvisio.core.model.DataNodeType;
import org.pathvisio.core.model.LineType;
import org.pathvisio.core.model.ObjectType;
import org.pathvisio.core.model.Pathway;
import org.pathvisio.core.model.PathwayElement;
import org.pathvisio.desktop.PvDesktop;
import pathlinker.Algorithms.Path;

public class Driver {
    private PvDesktop       desktop;
    private HashSet<String> visitedElements;
    private Pathway         pathway;


    public Driver(PvDesktop desk) {
        desktop = desk;
        visitedElements = new HashSet<>();
    }


    // rough setup. will modulize later
    public void
        buildSubgraphs(String sourceN, String targetN, String backgroundGraphFile, String pathNum)
            throws IOException {
        int k = Integer.parseInt(pathNum);
        String sourceNodes = sourceN.replace(',', ' ');
        String targetNodes = targetN.replace(',', ' ');
        long time = System.nanoTime();
        if(!desktop.getSwingEngine().canDiscardPathway()){
            return;
        }
        desktop.getSwingEngine().newPathway();
        pathway = desktop.getSwingEngine().getEngine().getActivePathway();
        // String set of sources and targets for ksp
        HashSet<String> sources = new HashSet<>();
        HashSet<String> targets = new HashSet<>();

        // read sources

        System.out.println(sourceNodes);
        Scanner input = new Scanner(sourceNodes);
        while(input.hasNext()){
            sources.add(input.next());
        }
        input.close();

        // read targets

        System.out.println(targetNodes);
        input = new Scanner(targetNodes);

        while(input.hasNext()){
            targets.add(input.next());
        }

        input.close();

        // creates graph with super sources and targets
        Graph graph = new DefaultGraph("mainGraph", false, false);
        Node superSource = graph.addNode("SOURCE");
        Node superTarget = graph.addNode("TARGET");

        // reads graph file
        BufferedReader reader = new BufferedReader(new FileReader(backgroundGraphFile));

        // set of initial hidden Edges
        HashSet<Edge> hiddenEdges = new HashSet<Edge>();

        // hash map of edge weights
        HashMap<Edge, Double> edgeWeights = new HashMap<>();

        // sum of all weights in graph. used for log transform
        double sumWeight = 0;
        String line;

        reader.readLine();
        while((line = reader.readLine()) != null){

            // adds nodes if not there and the corresponding edge between 2
            // nodes to the
            // graph
            input = new Scanner(line);

            Node start = graph.addNode(input.next());
            Node end = graph.addNode(input.next());
            double weight = input.nextDouble();
            StringBuilder sb = new StringBuilder();
            sb.append(start.getId());
            sb.append("->");
            sb.append(end.getId());
            Edge edge = graph.addEdge(sb.toString(), start, end, true);
            edge.addAttribute("weight", weight);

            // puts weight in edgeWeight map
            edgeWeights.put(edge, weight);

            // connects source to super source
            if(sources.contains(start.getId())){
                // adds edgeweight of 0 to edgemap
                edgeWeights.put(graph.addEdge("_" + start.getId(), superSource, start, true), 0.);
            }

            // connects target to super targets
            if(targets.contains(end.getId())){
                edgeWeights.put(graph.addEdge(end.getId() + "_", end, superTarget, true), 0.);
            }

            // adds all edges entering a source to hidden edges
            if(sources.contains(end.getId())){
                hiddenEdges.add(edge);
            }

            // adds all edges leaving a target to hidden edges
            else if(targets.contains(start.getId())){
                hiddenEdges.add(edge);
                // if edge isn't initially hidden add it to sumWeight
            }else{
                sumWeight = sumWeight + weight;
            }
        }

        reader.close();

        // log transform edgeWeights
        for(Edge edge : edgeWeights.keySet()){
            if(hiddenEdges.contains(edge))
                continue;

            double edge_weight = edgeWeights.get(edge);
            double w;
            if(edge_weight == 0){
                w = 0;
            }else{
                // add paramter later
                w = -1 * Math.log(Math.max(0.000000001, edge_weight)) / Math.log(10);
            }

            edgeWeights.put(edge, w);
        }

        // iniitializes pathlinker algorithims class
        Algorithms.initializeHiddenEdges(hiddenEdges);
        Algorithms.setEdgeWeights(edgeWeights);

        // runs pathlinker
        ArrayList<Algorithms.Path> result = Algorithms.ksp(graph, superSource, superTarget, k);

        Graph subgraph = new DefaultGraph("subgraph", false, false);
        HashSet<String> edges = new HashSet<>();
        for(Path p : result){
            makeSubgraph(p, subgraph, edges);
        }
        graph = subgraph;

        String text = printGraph(subgraph, sources, targets, pathway);
        System.out.println(System.nanoTime() - time);
        JOptionPane.showMessageDialog(desktop.getFrame(), text);
    }


    // want to try printing backwards?????
    // could have a better looking layout
    // will update for later
    private String printGraph(
        Graph subgraph,
        HashSet<String> sources,
        HashSet<String> targets,
        Pathway pathway) {
        // builds a string for summary
        StringBuilder sb = new StringBuilder();
        sb.append("Source Nodes found: ");
        Queue<Node> queue = new ArrayDeque<>();
        double x = 65;
        double y = 65;
        // print sources on top row
        for(String s : sources){
            Node n = subgraph.getNode(s);
            if(n == null){
                continue;
            }
            sb.append(s);
            sb.append(' ');
            queue.add(n);
            visitedElements.add(s);
            PathwayElement gnode = PathwayElement.createPathwayElement(ObjectType.DATANODE);
            gnode.setDataNodeType(DataNodeType.PROTEIN);
            gnode.setMCenterX(x);
            gnode.setMCenterY(65);
            gnode.setMHeight(20);
            gnode.setMWidth(80);
            gnode.setTextLabel(s);
            gnode.setElementID(s);
            gnode.setColor(Color.BLUE);
            n.addAttribute("X", x);
            n.addAttribute("Y", y);
            n.addAttribute("gNode", gnode);
            x = x + 100;
            pathway.add(gnode);
        }
        sb.append('\n');

        // reset x to 65. happens every level
        x = 65;
        // y coordinate for nodes
        y = 65;
        // print nodes in ordered format
        while(!queue.isEmpty()){
            Node prev = queue.remove();
            double newY = (Double)prev.getAttribute("Y");
            if(newY > y){
                y = newY;
                x = 65;
            }
            for(Edge e : prev.getEachLeavingEdge()){
                System.out.println(e.getSourceNode().getId() + ":" + e.getTargetNode().getId());
                Node curr = e.getTargetNode();
                if(!visitedElements.contains(curr.getId()) && !targets.contains(curr.getId())){
                    PathwayElement gnode = PathwayElement.createPathwayElement(ObjectType.DATANODE);
                    gnode.setDataNodeType(DataNodeType.PROTEIN);
                    gnode.setMCenterX(x);
                    gnode.setMCenterY(y + 100);
                    gnode.setMHeight(20);
                    gnode.setMWidth(80);
                    gnode.setTextLabel(curr.getId());
                    gnode.setElementID(curr.getId());
                    curr.addAttribute("X", x);
                    curr.addAttribute("Y", y + 100);
                    curr.addAttribute("gNode", gnode);
                    x = x + 100;
                    pathway.add(gnode);
                    visitedElements.add(curr.getId());
                    queue.add(curr);
                }
            }
        }

        x = 65;
        sb.append("Target Nodes found: ");
        for(String s : targets){
            Node curr = subgraph.getNode(s);
            if(curr == null){
                continue;
            }
            sb.append(s);
            sb.append(' ');
            visitedElements.add(s);
            PathwayElement gnode = PathwayElement.createPathwayElement(ObjectType.DATANODE);
            gnode.setDataNodeType(DataNodeType.PROTEIN);
            gnode.setMCenterX(x);
            gnode.setMCenterY(y + 100);
            gnode.setMHeight(20);
            gnode.setMWidth(80);
            gnode.setTextLabel(s);
            gnode.setElementID(s);
            gnode.setColor(Color.RED);
            curr.addAttribute("X", x);
            curr.addAttribute("Y", y + 100);
            x = x + 100;
            curr.addAttribute("gNode", gnode);
            pathway.add(gnode);
        }
        sb.append('\n');

        // print edges
        // want to move to above loop for efficiency.
        // not priority yet though
        for(Node n : subgraph){
            for(Edge e : n.getEachLeavingEdge()){
                if(!visitedElements.contains(e.getId())){
                    PathwayElement edge = PathwayElement.createPathwayElement(ObjectType.LINE);
                    edge.setEndLineType(LineType.ARROW);
                    edge.setMStartX((Double)e.getSourceNode().getAttribute("X"));
                    edge.setMStartY((Double)e.getSourceNode().getAttribute("Y") + 10);
                    edge.setMEndX((Double)e.getTargetNode().getAttribute("X"));
                    edge.setMEndY((Double)e.getTargetNode().getAttribute("Y") - 10);
                    edge.getMStart().linkTo(e.getSourceNode().getAttribute("gNode"));
                    edge.getMEnd().linkTo(e.getTargetNode().getAttribute("gNode"));
                    visitedElements.add(e.getId());
                    pathway.add(edge);
                }
            }
        }
        sb.append("Total nodes: ");
        sb.append(subgraph.getNodeCount());
        sb.append('\n');
        sb.append("Total Edges: ");
        sb.append(subgraph.getEdgeCount());
        return sb.toString();
    }


    private void makePath(Path p) {
        Node prev = null;
        double prevX = 0;
        double prevY = 0;
        for(Node n : p.nodeList){
            if(n.getId().equals("SOURCE") || n.getId().equals("TARGET")){
                continue;
            }

            if(!visitedElements.contains(n.getId())){
                PathwayElement node = PathwayElement.createPathwayElement(ObjectType.DATANODE);
                double x = Math.random() * 1000;
                node.setMCenterX(x);
                double y = Math.random() * 1000;
                node.setMCenterY(y);
                node.setMHeight(20);
                node.setMWidth(80);
                node.setTextLabel(n.getId());
                node.setElementID(n.getId());
                pathway.add(node);
                visitedElements.add(n.getId());
                n.addAttribute("X", x);
                n.addAttribute("Y", y);
            }

            if(prev != null){
                PathwayElement edge = PathwayElement.createPathwayElement(ObjectType.LINE);
                edge.setEndLineType(LineType.ARROW);
                edge.setMStartX(prevX);
                edge.setMStartY(prevY + 10);
                edge.setMEndX((Double)n.getAttribute("X"));
                edge.setMEndY((Double)n.getAttribute("Y") - 10);

                pathway.add(edge);
            }
            prev = n;
            prevX = n.getAttribute("X");
            prevY = n.getAttribute("Y");
        }

    }


    private void makeSubgraph(Path p, Graph subgraph, HashSet<String> edges) {
        Node prev = null;
        int count = 1;
        for(Node n : p.nodeList){
            if(n.getId().equals("SOURCE") || n.getId().equals("TARGET")){
                continue;
            }
            Node curr = subgraph.addNode(n.getId());
            if(!curr.hasAttribute("Path")){
                curr.addAttribute("Path", count);
            }
            if(prev != null){
                Edge edge = subgraph.addEdge(prev.getId() + "->" + curr.getId(), prev, curr, true);
                edges.add(edge.getId());
            }

            prev = curr;

        }

    }
}
