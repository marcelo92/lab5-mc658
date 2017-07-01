/*******************************************************
 * MC658 - Projeto e Analise de Algoritmo III - 1s2017
 * Prof: Flavio Keidi Miyazawa
 * PED: Edson Ticona Zegarra
 ******************************************************/
#include "prizecollectingpath.h"
 
///Preencher aqui para facilitar a correcao. 
// Nome1: Matheus Ferreira Tavares Boy
// RA1: 103501
// Nome2: Marcelo Abiarraj Teixeira
// RA2: 122128
 
///
// PLI function
///
int prize_collecting_st_path_pli(ListDigraph& g, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double>& cost, ListDigraph::Node s, ListDigraph::Node t, std::vector<ListDigraph::Node> &path, double &LB, double &UB, int tMax){
    std::vector<ListDigraph::Node> inv_path;
	GRBEnv env = GRBEnv(); 
	GRBModel model = GRBModel(env);
	GRBLinExpr exprNodes = 0.0;    
	GRBLinExpr exprArcs = 0.0;    
	model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    model.set(GRB_DoubleParam_TimeLimit, tMax);
	
    // define as variaveis binarias
    ListDigraph::ArcMap<GRBVar> x(g);
    for (ListDigraph::ArcIt e(g); e != INVALID; ++e) {
        x[e] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY);
    }
 
    ListDigraph::NodeMap<GRBVar> y(g);
    for (ListDigraph::NodeIt v(g); v != INVALID; ++v) {
        y[v] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY);
    }
 
    model.update();
    
    // constroi a funcao objetivo: max (soma dos custos do vertices - soma dos custos das arestas), sem considerar custo de s e t
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
 	  exprNodes += prize[v]*y[v];
    }
    for(ListDigraph::ArcIt e(g); e!=INVALID; ++e){
 	exprArcs -= cost[e]*x[e]; 
    }
    
    model.setObjective(exprNodes + exprArcs);
 
    //aplica as heuristicas de lower bound e upper bound
    prize_collecting_st_path_heuristic(g, prize, cost, s, t, path, LB, UB, tMax);
    model.addConstr(exprNodes + exprArcs >= LB);
    model.addConstr(exprNodes + exprArcs <= UB);
    model.update();
    
    //para todo vertice i!=s e i!=t, a soma das arestas que entram - soma das arestas que saem = 0
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
        if(v!=s && v!=t){
            GRBLinExpr exprin;
            for(ListDigraph::InArcIt e(g,v); e!=INVALID; ++e){
                exprin += x[e];
            }
 
            GRBLinExpr exprout;
            for(ListDigraph::OutArcIt e(g,v); e!=INVALID; ++e){
                exprout += x[e];
            }
            model.addConstr(exprin - exprout == 0);
        }
    }
 
    //para todo vertice v pertencente a V, Yv <= soma(arestas que saem de v)
    //essa restricao garante que nao haverao ciclos no caminho de s a t
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
        if(v!=s && v!=t){
            GRBLinExpr exprout;
            for(ListDigraph::OutArcIt e(g,v); e!=INVALID; ++e){
                exprout += x[e];
            }
            model.addConstr(y[v] >= exprout);
        }
    }
 
    //para v = s, a soma das arestas que entram - soma das arestas que saem = 1
    GRBLinExpr exprinS;
    for(ListDigraph::InArcIt e(g,s); e!=INVALID; ++e){
        exprinS += x[e];
    }
    GRBLinExpr exproutS;
    for(ListDigraph::OutArcIt e(g,s); e!=INVALID; ++e){
        exproutS += x[e];
    }
    model.addConstr(exproutS - exprinS == 1);
    
    //para v = t, a soma das arestas que entram - soma das arestas que saem = -1
    GRBLinExpr exprinT;
    for(ListDigraph::InArcIt e(g,t); e!=INVALID; ++e){
        exprinT += x[e];
    }
    GRBLinExpr exproutT;
    for(ListDigraph::OutArcIt e(g,t); e!=INVALID; ++e){
        exproutT += x[e];
    }
    model.addConstr(exproutT - exprinT == -1);
    
    model.update();
    model.write("teste.lp");
    model.optimize();
    
    // copia a solucao
   for(ListDigraph::NodeIt sol(g); sol!=INVALID; ++sol){
       GRBVar v = y[sol];
           if(v.get(GRB_DoubleAttr_X)==1){
		path.push_back(sol);
	   }
   }
   return true;
}
 
 
void quickSort(vector<ListDigraph::Node>& nodes, vector<ListDigraph::Arc>& arcs, ListDigraph::NodeMap<double>& prize, double left, double right) {
      double i = left, j = right;
      ListDigraph::Node swapNode;
      ListDigraph::Arc swapArc;
      double pivot = prize[nodes[(int)((left + right) / 2)]];
 
      /* partition */
      while (i <= j) {
            while (prize[nodes[i]] > pivot)
                  i++;
            while (prize[nodes[j]] < pivot)
                  j--;
            if (i <= j) {
                  swapNode = nodes[i];
                  nodes[i] = nodes[j] ;
                  nodes[j]  = swapNode;
                  swapArc = arcs[i];
                  arcs[i] = arcs[j];
                  arcs[j] = swapArc;
                  i++;
                  j--;
            }
      };
 
      /* recursion */
      if (left < j)
            quickSort(nodes, arcs, prize, left, j);
      if (i < right)
            quickSort(nodes, arcs, prize, i, right);
}
 
//dfs gulosa, pega sempre o caminho de maior premio, eh a heuristica de lower bound
int dfs(ListDigraph& g, vector<ListDigraph::Node> visited_nodes, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double>& cost, ListDigraph::Node current_node, ListDigraph::Node s, ListDigraph::Node t, int value, int* result){
	visited_nodes.push_back(current_node);
	if(current_node==t){
		if(value>*result){
			*result = value;
		}
		return value;
	}
 
	vector<ListDigraph::Node> nodes;
	vector<ListDigraph::Arc> arcs;
	for(ListDigraph::OutArcIt e(g,current_node); e!=INVALID; ++e){
		if(find(visited_nodes.begin(), visited_nodes.end(), g.target(e))==visited_nodes.end()){
			nodes.push_back(g.target(e));
			arcs.push_back(e);
		}
    }
    
    //ordena os vertices por valor do premio
    if(nodes.size()>0){
        quickSort(nodes, arcs, prize, 0l, (double)nodes.size()-1);
    }
 
	for(int i = 0; i<nodes.size(); i++){
		return dfs(g, visited_nodes,prize, cost, nodes[i], s, t, value+prize[nodes[i]]-cost[arcs[i]], result);
	}
	
	return  value;
 
}
 
// programa linear relaxado, serve de heuristica de upper bound
double pl(ListDigraph& g, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double>& cost, ListDigraph::Node s, ListDigraph::Node t, double &UB){
    	GRBEnv env = GRBEnv(); 
	GRBModel model = GRBModel(env);
	GRBLinExpr exprNodes = 0.0;    
	GRBLinExpr exprArcs = 0.0;    
	model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
	
    // define as variaveis binarias
    ListDigraph::ArcMap<GRBVar> x(g);
    for (ListDigraph::ArcIt e(g); e != INVALID; ++e) {
        x[e] = model.addVar(0.0, 1.0, 1.0, GRB_CONTINUOUS);
    }
 
    ListDigraph::NodeMap<GRBVar> y(g);
    for (ListDigraph::NodeIt v(g); v != INVALID; ++v) {
        y[v] = model.addVar(0.0, 1.0, 1.0, GRB_CONTINUOUS);
    }
 
    model.update();
    
    // constroi a funcao objetivo: max (soma dos custos do vertices - soma dos custos das arestas)
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
 	  exprNodes += prize[v]*y[v];
    }
    for(ListDigraph::ArcIt e(g); e!=INVALID; ++e){
 	exprArcs -= cost[e]*x[e]; 
    }
    
    model.setObjective(exprNodes + exprArcs);
    model.update();
    
    //para todo vertice i!=s e i!=t, a soma das arestas que entram - soma das arestas que saem = 0
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
        if(v!=s && v!=t){
            GRBLinExpr exprin;
            for(ListDigraph::InArcIt e(g,v); e!=INVALID; ++e){
                exprin += x[e];
            }
 
            GRBLinExpr exprout;
            for(ListDigraph::OutArcIt e(g,v); e!=INVALID; ++e){
                exprout += x[e];
            }
            model.addConstr(exprin - exprout == 0);
        }
    }
 
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
        if(v!=s && v!=t){
            GRBLinExpr exprout;
            for(ListDigraph::OutArcIt e(g,v); e!=INVALID; ++e){
                exprout += x[e];
            }
            model.addConstr(y[v] >= exprout);
        }
    }
 
    
    GRBLinExpr exprinS;
    for(ListDigraph::InArcIt e(g,s); e!=INVALID; ++e){
        exprinS += x[e];
    }
    GRBLinExpr exproutS;
    for(ListDigraph::OutArcIt e(g,s); e!=INVALID; ++e){
        exproutS += x[e];
    }
    model.addConstr(exproutS - exprinS == 1);
    
    GRBLinExpr exprinT;
    for(ListDigraph::InArcIt e(g,t); e!=INVALID; ++e){
        exprinT += x[e];
    }
    GRBLinExpr exproutT;
    for(ListDigraph::OutArcIt e(g,t); e!=INVALID; ++e){
        exproutT += x[e];
    }
    model.addConstr(exproutT - exprinT == -1);
    
    model.update();
    model.write("teste.lp");
    model.optimize();
    
    return model.get(GRB_DoubleAttr_ObjVal);
}
 
 
///
//
// Heuristic function
///
int prize_collecting_st_path_heuristic(ListDigraph& g, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double> &cost, ListDigraph::Node s, ListDigraph::Node t, std::vector<ListDigraph::Node> &path, double &LB, double &UB, int tMax){
    vector<ListDigraph::Node> visited_nodes;
	int result = 0;
	result = dfs(g, visited_nodes,prize, cost, s, s, t, 0, &result);
    LB = result;
    UB = pl(g, prize, cost, s, t, UB);
    
    // se as duas heuristicas encontraram um valor significativo, retorna sucesso
    if(LB > 0 && UB>0){
        return 1;
    }
	return 0;
}
 
 
