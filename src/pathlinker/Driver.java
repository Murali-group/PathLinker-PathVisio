package pathlinker;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Scanner;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.DefaultGraph;
import org.pathvisio.core.model.ObjectType;
import org.pathvisio.core.model.Pathway;
import org.pathvisio.core.model.PathwayElement;
import org.pathvisio.desktop.PvDesktop;
import pathlinker.Algorithms.Path;

public class Driver {
    private PvDesktop desktop;
    private HashSet<String> visitedNodes;
    private Pathway pathway;
    public Driver(PvDesktop desk) {
        desktop = desk;
        visitedNodes = new HashSet<>();
    }

    //rough setup.  will modulize later
    public void buildSubgraphs(String sourceNodes, String targetNodes, String backgroundGraphFile)
        throws IOException {
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
        Scanner input = new Scanner(sourceNodes);
        while(input.hasNext()){
            sources.add(input.next());
        }
        input.close();

        // read targets
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
                w = -1 * Math.log(Math.max(0.000000001, edge_weight / sumWeight)) / Math.log(10);
            }

            edgeWeights.put(edge, w);
        }

        // iniitializes pathlinker algorithims class
        Algorithms.initializeHiddenEdges(hiddenEdges);
        Algorithms.setEdgeWeights(edgeWeights);

        // runs pathlinker
        ArrayList<Algorithms.Path> result = Algorithms.ksp(graph, superSource, superTarget, 100);
        for(Path p : result) {
           makeNode(p);
        }
          System.out.println(System.nanoTime() - time);
    }

    private void makeNode(Path p) {
        Node prev = null;
        double prevX = 0;
        double prevY = 0;
        for(Node n : p.nodeList) {
            //will be removed when doing edges
            if(!visitedNodes.contains(n.getId())) {
                PathwayElement node = PathwayElement.createPathwayElement(ObjectType.DATANODE);
                double x = Math.random()*1000;
                node.setMCenterX(x);
                double y = Math.random()*1000;
                node.setMCenterY(y);
                node.setMHeight(20);
                node.setMWidth(80);
                node.setTextLabel(n.getId());
                node.setElementID(n.getId());
                pathway.add(node);
                visitedNodes.add(n.getId());
                n.addAttribute("X", x);
                n.addAttribute("Y", y);
            }

            if(prev!= null) {
                PathwayElement edge = PathwayElement.createPathwayElement(ObjectType.LINE);
                edge.setMStartX(prevX);
                edge.setMStartY(prevY);
                edge.setMEndX(n.getAttribute("X"));
                edge.setMEndY(n.getAttribute("Y"));
                pathway.add(edge);
            }
            prev = n;
            prevX = n.getAttribute("X");
            prevY = n.getAttribute("Y");
        }

    }
}
