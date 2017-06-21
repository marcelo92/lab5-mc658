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
        //printf("(%f) ", prize[g.target(e)]);
        if(g.target(e)!=s && g.target(e)!=t)
            expr += prize[g.target(e)]*x[e] - cost[e]*x[e];
        else
            expr += - cost[e]*x[e];
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
    
    for(ListDigraph::NodeIt v(g); v!=INVALID; ++v){
        if(v!=s && v!=t){
            GRBLinExpr exprin1;
            for(ListDigraph::InArcIt e(g,v); e!=INVALID; ++e){
                exprin1 += x[e];
            }
            model.addConstr(exprin1 <= 1);
            GRBLinExpr exprout1;
            for(ListDigraph::OutArcIt e(g,v); e!=INVALID; ++e){
                exprout1 += x[e];
            }
            model.addConstr(exprout1 <= 1);
        }
    }
    
    GRBLinExpr exprinS;
    for(ListDigraph::InArcIt e(g,s); e!=INVALID; ++e){
        exprinS += x[e];
    }
    model.addConstr(exprinS == 0);
    GRBLinExpr exproutS;
    for(ListDigraph::OutArcIt e(g,s); e!=INVALID; ++e){
        exproutS += x[e];
    }
    model.addConstr(exproutS == 1);
    
    GRBLinExpr exprinT;
    for(ListDigraph::InArcIt e(g,t); e!=INVALID; ++e){
        exprinT += x[e];
    }
    model.addConstr(exprinT == 1);
    GRBLinExpr exproutT;
    for(ListDigraph::OutArcIt e(g,t); e!=INVALID; ++e){
        exproutT += x[e];
    }
    model.addConstr(exproutT == 0);
    
    model.update();
    model.write("teste.lp");
    model.optimize();
    
    // copia a solucao
    ListDigraph::Node n = s;
    while(n!=t){
        for(ListDigraph::OutArcIt e(g,n); e!=INVALID; ++e){
            GRBVar v = x[e];
            if(v.get(GRB_DoubleAttr_X)==1.0){
                inv_path.push_back(g.source(e));
                n = g.target(e);
                break;
            }
        }
    }
    inv_path.push_back(t);
    
    for ( int i=(int)inv_path.size()-1; i>=0; i-- ){
        path.push_back(inv_path[i]);
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
