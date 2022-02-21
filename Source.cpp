#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string_view>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <optional>
#include <iomanip>
#include <set>
#include <tuple>
#include <utility>
#include "json.h"
#include "graph.h"
#include "router.h"

using namespace std;


const static double EARTH_RADIUS = 6371000.0;
const static double pi = acos(-1);

struct StopParams {
	double latitude = 0.0;
	double longitude = 0.0;
	unordered_multimap<string, double> distance = {};
	Graph::VertexId id_vertex = 0;
	Graph::VertexId mirror_id = 0;
	StopParams(double l, double r) : latitude((l * pi) / 180), longitude((r * pi) / 180) {};
	
};

struct BusInfo {
	size_t count = 0;
	size_t unique = 0;
	size_t length = 0;
	double curcvature = 0.0;
};


double ComputeLengthGeographic(const StopParams& lhs,const StopParams& rhs) {
	double l = acos(sin(lhs.latitude) * sin(rhs.latitude) + cos(lhs.latitude) * cos(rhs.latitude) * cos(abs(lhs.longitude - rhs.longitude))) * EARTH_RADIUS;
	return l;
}

double ComputeLength(const StopParams& lhs, const StopParams& rhs,
	const string& name_lhs, const string& name_rhs) {
	if (lhs.distance.find(name_rhs) != lhs.distance.end()) {
		return lhs.distance.find(name_rhs)->second;
	}
	else {
		return rhs.distance.find(name_lhs)->second;
	}
}

struct EdgeInfo {
	optional<string> bus_name = nullopt;
	string type;
	optional<string> stop_name = nullopt;
	double time;
	optional<size_t> span_count = nullopt;
};

ostream& operator<<(ostream& str, const EdgeInfo& edge) {
	str << "{" << endl;
	str << R"("time": )" << edge.time << "," << endl;
	if (edge.bus_name) {
		str << R"("bus": )" << "\"" << edge.bus_name.value() << "\"" << "," << endl;
		str << R"("span_count": )"  << edge.span_count.value() << "," << endl;
		str << R"("type": )" << "\"" << edge.type << "\"" << endl;
	}
	else {
		str << R"("stop_name": )" << "\"" << edge.stop_name.value() << "\"" << "," << endl;
		str << R"("type": )" << "\"" << edge.type << "\"" << endl;
	}
	str << "}";
	return str;
}

class BusManager {
public:
	void AddStop(string name, double lat, double lon, const vector<pair<string, size_t>>& dis) {
		StopParams st(lat, lon);
		for (const auto& [distance_to_name, distance_to_meters]: dis) {
			st.distance.insert({ distance_to_name, static_cast<double>(distance_to_meters) });
		}
		stops.insert({ name, st });
	}

	void AddBus(string num, const vector<string>& st_names, bool f) {
		data[num].collection.reserve(st_names.size());
		for (const auto& c : st_names) {
			data[num].collection.push_back(c);
			data[num].unique_stops.insert(c);
		}
		data[num].is_round_trip = f;
	}

	void AddSettings(size_t bus_time, size_t bus_vel) {
		settings = { static_cast<double>(bus_time), static_cast<double>(bus_vel) };
	}

	optional<BusInfo> BusInformation(const string& num) const {
		BusInfo info;
		if (data.find(num) == data.end()) {
			return nullopt;
		}
		info.count = data.at(num).collection.size();
		info.unique = data.at(num).unique_stops.size();
		const auto& names = data.at(num).collection;
		double length_GEO = 0.0;
		double distance = 0;
		for (size_t i = 0; i < names.size(); i++) {
			if (i + 1 >= names.size()) {
				break;
			}
			auto lhs = stops.find(names.at(i));
			auto rhs = stops.find(names.at(i + 1));
			
			distance += ComputeLength(lhs->second, rhs->second, lhs->first, rhs->first); // segfault
			length_GEO += ComputeLengthGeographic(lhs->second, rhs->second);
		}
		info.length = static_cast<int>(distance);
		info.curcvature = distance / length_GEO;
		return  info;
	}

	optional<set<string>> StopInformation(const string& name) const {
		set<string> result;
		if (stops.find(name) == stops.end()) {
			return nullopt;
		}
		for (const auto&[bus_num, stops] : data) {
			if (stops.unique_stops.find(name) != stops.unique_stops.end()) {
				result.insert(bus_num);
			}
		}
		return result;
	}
	size_t GetStopId(const string& name) const {
		return stops.find(name)->second.id_vertex;
	}
	size_t GetMirId(const string& name) const {
		return stops.find(name)->second.mirror_id;
	}
	struct IdEdgeAndBuses{
		size_t id = 0;
		vector<string> buses_name_on_id;
	};

	Graph::DirectedWeightedGraph<double> BuildGraph() {
		Graph::DirectedWeightedGraph<double> res(stops.size()*2 + 1);
		size_t tmp = 0;
		size_t pos = 0;
		for (auto it = stops.begin(); it != stops.end(); it++) {
			if (next(it) == stops.end()) {
				it->second.mirror_id = ++tmp + pos + 1;
				break;
			}
			pos++;
			it->second.id_vertex = tmp + pos;
			it->second.mirror_id = ++tmp + pos;
			next(it)->second.id_vertex = tmp + pos + 1;
		}
		for (auto it = stops.begin(); it != stops.end(); it++) {
			Graph::Edge<double> wait = { it->second.id_vertex, it->second.mirror_id, settings.wait_time };
			EdgeInfo info;
			info.type = "Wait";
			info.stop_name = it->first;
			info.time = settings.wait_time;
			size_t id_e = res.AddEdge(wait);
			edges.insert({ id_e, info });
		}
		vector<double> r;
		for (const auto&[b, st] : data) {
				for (size_t i = 0; i < st.collection.size(); i++) {
					double time = 0.0;
					auto from = stops.find(st.collection[i]);
					for (size_t j = i + 1; j < st.collection.size(); j++) {
						time = ComputeForwardLength(i, j, b) / 1000.0 / settings.bus_velocity  * 60.0;
						/*if (time == 0) {
							time = (ComputeCircularLength(i, j, b) / 1000.0 / settings.bus_velocity * 60.0) + settings.wait_time;
						}*/
						Graph::Edge<double> bus;
						auto to = stops.find(st.collection[j]);
						bus = { from->second.mirror_id, to->second.id_vertex, time };
						EdgeInfo info;
						info.type = "Bus";
						info.bus_name = b;
						if (j - i == 0 || i - j == 0) {
							info.span_count = st.collection.size();
						}
						else if (j > i) {
							info.span_count = j - i;
						}
						else {
							info.span_count = i - j;
						}
						info.time = time;
						size_t id_e = res.AddEdge(bus);
						edges.insert({ id_e, info });
					}
				}
		}
		return res;
	}
	
	EdgeInfo InfoByEdge(size_t edgeid) const {
		return edges.at(edgeid);
	}

	struct AboutBus {
		vector<string> collection = {};
		unordered_set<string> unique_stops = {};
		bool is_round_trip;
	};
	
	struct RoutingSettings {
		double wait_time = 0;
		double bus_velocity = 0;
	};


private:
	unordered_map<Graph::EdgeId, EdgeInfo> edges;
	RoutingSettings settings;
	unordered_map<string, AboutBus> data = {};
	unordered_multimap<string, StopParams> stops = {};
	double ComputeCircularLength(size_t from, size_t to, const string& bus) const {
		const auto& r = data.at(bus).collection;
		double result = 0.0;
		for (size_t i = from; i < r.size(); i++) {
			if (i + 1 >= r.size()) {
				break;
			}
			auto lhs = stops.find(r.at(i));
			auto rhs = stops.find(r.at(i + 1));
			result += ComputeLength(lhs->second, rhs->second, lhs->first, rhs->first);
		}
		for (size_t i = 0; i < to; i++) {
			if (i + 1 >= r.size() || i + 1 > to) {
				break;
			}
			auto lhs = stops.find(r.at(i));
			auto rhs = stops.find(r.at(i + 1));
			result += ComputeLength(lhs->second, rhs->second, lhs->first, rhs->first);
		}
		return result;
	}

	double ComputeForwardLength(size_t from, size_t to, const string& bus) const {
		const auto& names = data.at(bus).collection;
		double distance = 0;
		for (size_t i = from; i < to; i++) {
			if (i + 1 >= names.size()) {
				break;
			}
			auto lhs = stops.find(names.at(i));
			auto rhs = stops.find(names.at(i + 1));

			distance += ComputeLength(lhs->second, rhs->second, lhs->first, rhs->first); // segfault
		}
		return distance;
	}
};

void SetRouteSettingsForBase(const map<string, Json::Node>& arr, BusManager& db) {
	int vel = arr.at("bus_velocity").AsInt();
	int time = arr.at("bus_wait_time").AsInt();
	db.AddSettings(time, vel);
}


void ParseNodeArrayInput(const vector<Json::Node>& arr, BusManager& db) {
	for (const auto& r : arr) {
		const auto& my_map = r.AsMap();
		const auto& req = my_map.at("type"s).AsString();
		if (req == "Stop") {
			vector<pair<string, size_t>> road_dist;
			double lat, lon;
			string name;
			name = my_map.at("name"s).AsString();
			lat = my_map.at("latitude"s).AsDouble();
			lon = my_map.at("longitude"s).AsDouble();
			for (const auto& [n, value] : my_map.at("road_distances"s).AsMap()) {
				road_dist.push_back({ n, value.AsInt() });
			}
			db.AddStop(name, lat, lon, road_dist);
		}
		else {
			string name;
			vector<string> stops_name;
			name = my_map.at("name"s).AsString();
			const auto& node_names = my_map.at("stops"s).AsArray();
			for (const auto& n : node_names) {
				stops_name.push_back(n.AsString());
			}
			if (!my_map.at("is_roundtrip"s).AsBool()) {
				stops_name.reserve(2 * stops_name.size() - 1);
				stops_name.insert(stops_name.end(), stops_name.rbegin() + 1, stops_name.rend());
			}
			db.AddBus(name, stops_name, my_map.at("is_roundtrip"s).AsBool());
		}
	}
	
}
struct OutputStop {
	int id = 0;
	optional<set<string>> buses = {};
};

ostream& operator<<(ostream& str, const OutputStop& stops) {
	str << "{";
	str << R"("request_id": )" << stops.id << ",";
	if (stops.buses) {
		str << R"("buses": [)";
		size_t size = stops.buses.value().size();
		if (size > 0){
			for (auto it = stops.buses.value().begin(); it != stops.buses.value().end(); it++) {
				if (it == prev(stops.buses.value().end())) {
					str << "\"" << *it << "\"";
					break;
				}
				str << "\"" << *it << "\"" << ",";
			}
		}
		str << "]";
	}
	else {
		str << R"("error_message": "not found" )";
	}
	str << "}";
	return str;
}

struct OutputBus {
	int id = 0;
	optional<BusInfo> info = {};
};
ostream& operator<<(ostream& str, const OutputBus& buses) {
	str << "{";
	str << R"("request_id": )" << buses.id << ",";
	if (buses.info) {
		str << R"("route_length": )" << buses.info.value().length << ",";
		str << R"("curvature": )" << buses.info.value().curcvature << ",";
		str << R"("stop_count": )" << buses.info.value().count << ",";
		str << R"("unique_stop_count": )" << buses.info.value().unique;
	}
	else {
		str << R"("error_message": "not found")";
	}
	str << "}";
	return str;
}

struct OutputRoute {
	int id = 0;
	optional<vector<EdgeInfo>> info_about_route;
	optional<double> total_time;
};

ostream& operator<<(ostream& str, const OutputRoute& route) {
	str << "{" << endl;
	if (route.info_about_route) {
		str << "\"" << "items" << "\"" << ": [" << endl;
		for (size_t i = 0; i < route.info_about_route.value().size(); i++) {
			if (i == route.info_about_route.value().size() - 1) {
				str << route.info_about_route.value()[i];
				break;
			}
			str << route.info_about_route.value()[i] << "," << endl;
		}
		str << "]" << "," << endl;
		str << "\"" << "request_id" << "\"" << ": " << route.id << "," << endl;
		str << "\"" << "total_time" << "\"" << ": " << route.total_time.value() << endl;
	}
	else {
		str << R"("error_message": "not found" )" << ", " << endl;
		str << "\"" << "request_id" << "\"" << ": " << route.id << endl;
	}
	str << "}";
	return str;
}
void PrintJsonOut(const vector<OutputStop>& out_st, const vector<OutputBus>& out_bus,  const vector<OutputRoute>& out_route) {
	cout << "[";
	for (const auto& c : out_bus) {
		cout << c << ",";
	}
	for (size_t i = 0; i < out_st.size(); i++) {
		cout << out_st[i] << ",";
	}
	for (size_t i = 0; i < out_route.size(); i++) {
		if (i == out_route.size() - 1) {
			cout << out_route[i];
			break;
		}
		cout << out_route[i] << ",";
	}
	cout << "]";
}

void ParseNodeArrayDataBaseQuery(const vector<Json::Node>& arr,  BusManager& db) {
	vector<OutputStop> out_st = {};
	vector<OutputBus> out_bus = {};
	vector<OutputRoute> info_about_route = {};
	auto c = db.BuildGraph();
	Graph::Router<double> route(c);
	for (const auto& r : arr) {
		auto my_map = r.AsMap();
		auto req = my_map.at("type").AsString();
		if (req == "Stop") {
			auto inf = db.StopInformation(my_map.at("name").AsString());
			out_st.push_back(OutputStop{ my_map.at("id").AsInt(), move(inf) });
		}
		else if(req == "Bus"){
			auto inf = db.BusInformation(my_map.at("name").AsString());
			out_bus.push_back(OutputBus{ my_map.at("id").AsInt(), move(inf) });
		}
		else {
			auto info = route.BuildRoute(db.GetStopId(my_map.at("from").AsString()), 
				db.GetStopId(my_map.at("to").AsString()));
			if (info) {
				size_t edgeid;
				vector<EdgeInfo> r;
				for (size_t i = 0; i < info.value().edge_count; i++) {
					edgeid = route.GetRouteEdge(info.value().id, i);
					r.push_back(db.InfoByEdge(edgeid));
				}
				info_about_route.push_back(OutputRoute{ my_map.at("id").AsInt(), move(r), info.value().weight});
			}
			else {
				info_about_route.push_back(OutputRoute{ my_map.at("id").AsInt(), nullopt, nullopt });
			}
		}
	}
	PrintJsonOut(out_st, out_bus, info_about_route);
}

void TestGet() {
	istringstream str("0.000001");
	Json::Document n = Json::Load(str);
	istringstream str1("1");
	Json::Document n1 = Json::Load(str1);
	istringstream str2("-0.01");
	Json::Document n2 = Json::Load(str2);
	istringstream str3("-1.5");
	Json::Document n3 = Json::Load(str3);
	auto a = n.GetRoot().AsDouble();
	auto b = n1.GetRoot().AsDouble();
	auto c = n2.GetRoot().AsDouble();
	auto d = n3.GetRoot().AsDouble();
	cout << a;
	string s = "dsf";
}

int main() {
	BusManager db;
	//TestGet();
	auto request = Json::Load(cin).GetRoot().AsMap();;
	for (const auto [name, node] : request) {
		if (name == "base_requests") {
			ParseNodeArrayInput(node.AsArray(), db);
		}
		else if (name == "routing_settings") {
			SetRouteSettingsForBase(node.AsMap(), db);
		}
		else if(name == "stat_requests"){
			ParseNodeArrayDataBaseQuery(node.AsArray(), db);
		}
	}
//#define Min(x) (x) / 1000.0 / 30.0 * 60.0	
//	vector<Graph::Edge<double>> p = {
//		{0,1,6},
//	{4,5,6},
//	{0,5,Min(2600 + 890)},
//	{5,1,Min(2600)},
//	{}
//	};
//
//	Graph::DirectedWeightedGraph<double> gr(10);
//	
//	for (const auto& c : p) {
//		gr.AddEdge(c);
//	}
//
//	Graph::Router<double> route(gr);
//	auto inf = route.BuildRoute(0, 4);
//	inf;
//	auto fin = route.GetRouteEdge(inf->id, 1);
//	fin;
	return 0;
}
