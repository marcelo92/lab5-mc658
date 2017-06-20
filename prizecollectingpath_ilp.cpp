/*******************************************************
 * MC658 - Projeto e Analise de Algoritmo III - 1s2017
 * Prof: Flavio Keidi Miyazawa
 * PED: Edson Ticona Zegarra
 ******************************************************/
#include "prizecollectingpath.h"

///Preencher aqui para facilitar a correcao. 
// Nome1:
// RA1:
// Nome2: 
// RA2:

///
// PLI function
///
int prize_collecting_st_path_pli(ListDigraph& g, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double>& cost, ListDigraph::Node s, ListDigraph::Node t, std::vector<ListDigraph::Node> &path, double &LB, double &UB, int tMax){
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	GRBLinExpr expr = 0.0;    
	model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
	env.set(GRB_DoubleParam_TimeLimit, tMax);
	
    // define as variaveis binarias
    ListDigraph::ArcMap<GRBVar> x(g);
    for (ListDigraph::ArcIt e(g); e != INVALID; ++e) {
        x[e] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY);
    }
    model.update();
    
     // constroi a funcao objetivo: max (soma dos custos do vertices - soma dos custos das arestas)
    for(ListDigraph::ArcIt e(g); e!=INVALID; ++e){
        printf("(%f) ", prize[g.target(e)]);
        expr += prize[g.source(e)]*x[e] - cost[e]*x[e];
    }
    model.setObjective(expr);
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
    
    // para i=s, soma das arestas que entram - soma das arestas que saem = 1
    GRBLinExpr exprinS;
    for(ListDigraph::InArcIt e(g,s); e!=INVALID; ++e){
        exprinS += x[e];
    }
            
    GRBLinExpr exproutS;
    for(ListDigraph::OutArcIt e(g,s); e!=INVALID; ++e){
        exproutS += x[e];
    }
    model.addConstr(exprinS - exproutS == 1);
    
    // para i=t, soma das arestas que entram - soma das arestas que saem = 1
    GRBLinExpr exprinT;
    for(ListDigraph::InArcIt e(g,t); e!=INVALID; ++e){
        exprinT += x[e];
    }
            
    GRBLinExpr exproutT;
    for(ListDigraph::OutArcIt e(g,t); e!=INVALID; ++e){
        exproutT += x[e];
    }
    model.addConstr(exprinT - exproutT == -1);
    
    model.update();
    model.write("teste.lp");
    model.optimize();
    
    // copia a solucao
    for (ListDigraph::ArcIt e(g); e != INVALID; ++e) {
        GRBVar v = x[e];
        if(v.get(GRB_DoubleAttr_X)!=0.0){
            path.push_back(g.source(e));
        }
    }

	return true;
}


///
//
// Heuristic function
///
int prize_collecting_st_path_heuristic(ListDigraph& g, ListDigraph::NodeMap<double>& prize, ListDigraph::ArcMap<double> &cost, ListDigraph::Node s, ListDigraph::Node t, std::vector<ListDigraph::Node> &path, double &LB, double &UB, int tMax){
	return 0;
}
