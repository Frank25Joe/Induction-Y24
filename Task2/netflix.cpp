#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <limits>

class user;
class admin;
class content;
class movie;
class TVshow;
class mediaLibrary;
void welcome();
void user_page(user*, mediaLibrary&);
void admin_page(admin*, mediaLibrary&);

std::vector<user> user_list{};

std::string to_lowercase(const std::string& s){
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

class content{
    public:
        std::string title;
        std::string genre;
        double rating;
};

class movie : public content{
    public:
        int duration; //in minutes
        double rent_cost;
        double purchase_cost;
};

class TVshow : public content{
    public:
        int seasons;
        int episodes_per_season;
        double season_rent_cost;
        double season_purchase_cost;
};

struct rented_movie{
    std::string title;
    std::time_t date_rented;
    std::time_t return_date;    
};

struct rented_TVshow{
    std::string title;
    int num_seasons_rented;
    std::time_t date_rented;
    std::time_t return_date;
};

struct purchased_TVshow{
    std::string title;
    int num_seasons_purchased;
};

class mediaLibrary{
    private:
        std::vector<movie> movie_list{};
        std::vector<TVshow> TVshow_list{};
    friend class admin;
    public:
        const std::vector<movie>& get_movies() const{
            return movie_list;
        }
        const std::vector<TVshow>& get_TVshows() const{
            return TVshow_list;
        }

        void save_movie_data(){
            std::ofstream fout("movie_list.txt");
            fout << movie_list.size() << "\n";
            for(const auto& m : movie_list){
                fout << m.title << "\n";
                fout << m.genre << "\n";
                fout << m.rating << " " << m.duration << " " << m.rent_cost << " " << m.purchase_cost << "\n";
            }
            fout.close();
        }

        void load_movie_data(){
            std::ifstream fin("movie_list.txt");
            if(!fin) return;
            size_t num_movies;
            fin >> num_movies;
            movie_list.clear();
            for(size_t i = 0; i < num_movies; i++){
                movie m;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, m.title);
                std::getline(fin, m.genre);
                fin >> m.rating >> m.duration >> m.rent_cost >> m.purchase_cost;
                movie_list.push_back(m);
            }
            fin.close();
        }

        void save_TVshow_data(){
            std::ofstream fout("TVshow_list.txt");
            fout << TVshow_list.size() << "\n";
            for(const auto& t : TVshow_list){
                fout << t.title << "\n";
                fout << t.genre << "\n";
                fout << t.rating << " " << t.seasons << " " << t.episodes_per_season << " " << t.season_rent_cost << " " << t.season_purchase_cost << "\n";
            }
            fout.close();
        }

        void load_TVshow_data(){
            std::ifstream fin("TVshow_list.txt");
            if(!fin) return;
            size_t num_TVshows;
            fin >> num_TVshows;
            TVshow_list.clear();
            for(size_t i = 0; i < num_TVshows; i++){
                TVshow t;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, t.title);
                std::getline(fin, t.genre);
                fin >> t.rating >> t.seasons >> t.episodes_per_season >> t.season_rent_cost >> t.season_purchase_cost;
                TVshow_list.push_back(t);
            }
            fin.close();
        }

        bool movie_exists(std::string title){
            for(const auto& m : movie_list){
                std::string lower_title = to_lowercase(m.title);
                if(lower_title == title){
                    std::cout << m.title << " already exists.\n";
                    return true;
                }
            }
            return false;
        }
        bool TVshow_exists(std::string title){
            for(const auto& t : TVshow_list){
                std::string lower_title = to_lowercase(t.title);
                if(lower_title == title){
                    std::cout << t.title << " already exists.\n";
                    return true;
                }
            }
            return false;
        }
        void search_by_title(const std::string& keyword) const{
            std::string key = to_lowercase(keyword);
            bool found = false;
            for(const auto& m : movie_list){
                std::string lower_title = to_lowercase(m.title);
                if(lower_title.find(key) != std::string::npos){
                    std::cout << m.title << " | " << m.genre << " | Rating: " << m.rating << " | " << m.duration << " mins | Rent cost: " << m.rent_cost << " | Purchase cost: " << m.purchase_cost << "\n";
                    found = true;
                }
            }
            if(!found){
                std::cout << "No movies found.\n";
            }
            found = false;
            for(const auto& t : TVshow_list){
                std::string lower_title = to_lowercase(t.title);
                if(lower_title.find(key) != std::string::npos){
                    std::cout << t.title << " | " << t.genre << " | Rating: " << t.rating << " | " << t.episodes_per_season << " episodes per season, " << t.seasons << " seasons | Rent cost per season: " << t.season_rent_cost << " | Purchase cost per season: " << t.season_purchase_cost << "\n";
                    found = true;
                }
            }
            if(!found){
                std::cout << "No TV shows found.\n";
            }
        }
        void search_by_genre(const std::string& requested_genre) const {
            std::string reqd_genre = to_lowercase(requested_genre);
            bool found = false;
            for (const auto& m : movie_list) {
                if (to_lowercase(m.genre) == reqd_genre) {
                    std::cout << m.title << " | " << m.genre << " | Rating: " << m.rating << " | " << m.duration << " mins | Rent cost: " << m.rent_cost << " | Purchase cost: " << m.purchase_cost << "\n";
                    found = true;
                }
            }
            if (!found) std::cout << "No movies found in genre \"" << reqd_genre << "\".\n";
            found = false;
            for (const auto& t : TVshow_list) {
                if (to_lowercase(t.genre) == reqd_genre) {
                    std::cout << t.title << " | " << t.genre << " | Rating: " << t.rating << " | " << t.episodes_per_season << " episodes per season, " << t.seasons << " seasons | Rent cost per season: " << t.season_rent_cost << " | Purchase cost per season: " << t.season_purchase_cost << "\n";
                    found = true;
                }
            }
            if (!found) std::cout << "No TV shows found in genre \"" << reqd_genre << "\".\n";
        }
    private:
        void add_movie(const movie& m){
            movie_list.push_back(m);
            save_movie_data();
            std::cout << "Added " << m.title << " successfully.\n";
        }
        void add_TVshow(const TVshow& t){
            TVshow_list.push_back(t);
            save_TVshow_data();
            std::cout << "Added " << t.title << " successfully.\n";
        }
        bool remove_movie(const std::string& title){
            for(auto i = movie_list.begin(); i!=movie_list.end(); i++){
                if(i->title == title){
                    movie_list.erase(i);
                    save_movie_data();
                    return true;
                }
            }
            return false;
        }
        bool remove_TVshow(const std::string& title){
            for(auto i = TVshow_list.begin(); i!=TVshow_list.end(); i++){
                if(i->title == title){
                    TVshow_list.erase(i);
                    save_TVshow_data();
                    return true;
                }
            }
            return false;
        }
};

class user{
    friend void signup();
    friend user* login();
    private:
        std::string password;
    public:
        std::string username;
        double charges_due;
        std::vector<rented_movie> rented_movies;
        std::vector<std::string> purchased_movies;
        std::vector<rented_TVshow> rented_TVshows;
        std::vector<purchased_TVshow> purchased_TVshows;
        user(){
            charges_due = 0.00;
            rented_movies = {};
            purchased_movies = {};
        }

        void save_user_data() const{
            std::ofstream fout(username + ".txt");
            fout << username << "\n";
            fout << password << "\n";
            fout << charges_due << "\n";
            fout << rented_movies.size() << "\n";
            for (const auto& rm : rented_movies){
                fout << rm.title << "\n";
                fout << rm.date_rented << " " << rm.return_date << "\n";
            }
            fout << rented_TVshows.size() << "\n";
            for (const auto& rt : rented_TVshows){
                fout << rt.title << "\n";
                fout << rt.num_seasons_rented << " " << rt.date_rented << " " << rt.return_date << "\n";
            }
            fout << (purchased_movies).size() << "\n";
            for (const auto& pm : purchased_movies){
                fout << pm << "\n";
            }
            fout << purchased_TVshows.size() << "\n";
            for (const auto& pt : purchased_TVshows){
                fout << pt.title << "\n";
                fout << pt.num_seasons_purchased << "\n";
            }
            fout.close();
        }

        void load_user_data(){
            std::ifstream fin(username + ".txt");
            if(!fin) return;
            std::getline(fin, username);
            std::getline(fin, password);
            fin >> charges_due;
            size_t num_rented_movies, num_rented_TVshows, num_purchased_movies, num_purchased_TVshows;
            
            fin >> num_rented_movies;
            rented_movies.clear();
            for(size_t i = 0; i < num_rented_movies; i++){
                rented_movie rm;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, rm.title);
                fin >> rm.date_rented >> rm.return_date;
                rented_movies.push_back(rm);
            }
            
            fin >> num_rented_TVshows;
            rented_TVshows.clear();
            for(size_t i = 0; i < num_rented_TVshows; i++){
                rented_TVshow rt;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, rt.title);
                fin >> rt.num_seasons_rented >> rt.date_rented >> rt.return_date;
                rented_TVshows.push_back(rt);
            }

            fin >> num_purchased_movies;
            purchased_movies.clear();
            for(size_t i = 0; i < num_purchased_movies; i++){
                std::string title;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, title);
                purchased_movies.push_back(title);
            }
            
            fin >> num_purchased_TVshows;
            purchased_TVshows.clear();
            for(size_t i = 0; i < num_purchased_TVshows; i++){
                purchased_TVshow pt;
                fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::getline(fin, pt.title);
                fin >> pt.num_seasons_purchased;
                purchased_TVshows.push_back(pt);
            }
            fin.close();
        }

        void check_charges(){
            std::cout << "You have Rs. " << charges_due << " due.\n";
        }
        void view_movies(const mediaLibrary& lib){
            const auto& movies = lib.get_movies();
            if(movies.empty()){
                std::cout << "No movies found.\n";
            }
            else{
                for(const auto& m : movies){
                    std::cout << m.title << " | " << m.genre << " | Rating: " << m.rating << " | " << m.duration << " mins | Rent cost: " << m.rent_cost << " | Purchase cost: " << m.purchase_cost << "\n";
                }
            }
        }
        void view_TVshows(const mediaLibrary& lib){
            const auto& TVshows = lib.get_TVshows();
            if(TVshows.empty()){
                std::cout << "No TV shows found.\n";
            }
            else{
                for(const auto& t : TVshows){
                    std::cout << t.title << " | " << t.genre << " | Rating: " << t.rating << " | " << t.episodes_per_season << " episodes per season, " << t.seasons << " seasons | Rent cost per season: " << t.season_rent_cost << " | Purchase cost per season: " << t.season_purchase_cost << "\n";
                }
            }
        }
        void rent_movie(const std::string& title, const mediaLibrary& lib) {
            for (const auto& m : lib.get_movies()) {
                if (to_lowercase(m.title) == to_lowercase(title)) {
                    charges_due += m.rent_cost;
                    rented_movie r_m;
                    r_m.title = m.title;
                    r_m.date_rented = std::time(nullptr);
                    r_m.return_date = r_m.date_rented + (3*24*3600);
                    rented_movies.push_back(r_m);
                    std::cout << "You rented \"" << m.title << "\" for Rs. " << m.rent_cost << "\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "Movie \"" << title << "\" not found.\n";
        }
        void rent_TVshow(const std::string& title, int num_seasons_rented, const mediaLibrary& lib){
            for (const auto& t : lib.get_TVshows()) {
                if (to_lowercase(t.title) == to_lowercase(title)){
                    if(num_seasons_rented > t.seasons){
                        std::cout << t.title << " has only " << t.seasons << " seasons.\n";
                        return;
                    }
                    charges_due += num_seasons_rented * t.season_rent_cost;
                    rented_TVshow r_t;
                    r_t.title = t.title;
                    r_t.num_seasons_rented = num_seasons_rented;
                    r_t.date_rented = std::time(nullptr);
                    r_t.return_date = r_t.date_rented + (3*24*3600); 
                    rented_TVshows.push_back(r_t);
                    std::cout << "You rented " << num_seasons_rented << " seasons of \"" << t.title << "\" for a total of Rs. " << num_seasons_rented * t.season_rent_cost << "\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "TV show \"" << title << "\" not found.\n";
        }
        void purchase_movie(const std::string& title, const mediaLibrary& lib) {
            for (const auto& m : lib.get_movies()) {
                if (to_lowercase(m.title) == to_lowercase(title)){
                    charges_due += m.purchase_cost;
                    purchased_movies.push_back(m.title);
                    std::cout << "You purchased \"" << m.title << "\" for Rs. " << m.purchase_cost << "\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "Movie \"" << title << "\" not found.\n";
        }
        void purchase_TVshow(const std::string& title, int num_seasons_purchased, const mediaLibrary& lib) {
            for (const auto& t : lib.get_TVshows()) {
                if (to_lowercase(t.title) == to_lowercase(title)){
                    if(num_seasons_purchased > t.seasons){
                        std::cout << t.title << " has only " << t.seasons << " seasons.\n";
                        return;
                    }
                    charges_due += num_seasons_purchased * t.season_purchase_cost;
                    purchased_TVshow p_t;
                    p_t.title = t.title;
                    p_t.num_seasons_purchased = num_seasons_purchased;
                    purchased_TVshows.push_back(p_t);
                    std::cout << "You purchased " << num_seasons_purchased << " seasons of \"" << t.title << "\" for a total of Rs. " << num_seasons_purchased * t.season_purchase_cost << "\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "TV show \"" << title << "\" not found.\n";
        }
        void view_purchased_items() const{
            std::cout << "\n---Purchased movies---\n";
            if(purchased_movies.empty()){
                std::cout << "No purchased movies.\n";
            }
            else{
                for(const auto& title : purchased_movies){
                    std::cout << title << "\n";
                }
            }
            std::cout << "\n---Purchased TV shows---\n";
            if(purchased_TVshows.empty()){
                std::cout << "No purchased TV shows.\n";
            }
            else{
                for(const auto& p_t : purchased_TVshows){
                    std::cout << p_t.title << " " << "(" << p_t.num_seasons_purchased << " seasons purchased)" << "\n";
                }
            }
        }
        void view_rented_items() const{
            std::cout << "\n---Rented movies---\n";
            if(rented_movies.empty()){
                std::cout << "No rented movies.\n";
            }
            else{
                for(const auto& r_m : rented_movies){
                    std::cout << r_m.title << "\n";
                    std::cout << "Rented on: " << std::ctime(&r_m.date_rented);
                    std::cout << "Last date of return: " << std::ctime(&r_m.return_date) << "\n";
                }
            }
            std::cout << "\n---Rented TV shows---\n";
            if(rented_TVshows.empty()){
                std::cout << "No rented TV shows.\n";
            }
            else{
                for(const auto& r_t : rented_TVshows){
                    std::cout << r_t.title << " " << "(" << r_t.num_seasons_rented << " seasons rented)" << "\n";
                    std::cout << "Rented on: " << std::ctime(&r_t.date_rented);
                    std::cout << "Last date of return: " << std::ctime(&r_t.return_date) << "\n";
                }
            }
        }
        void return_movie(const std::string& title){
            for(auto i = rented_movies.begin(); i!=rented_movies.end(); i++){
                if(to_lowercase(i->title) == to_lowercase(title)){
                    rented_movies.erase(i);
                    std::cout << "Movie " << title << " returned successfully.\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "Movie " << title << " not rented.\n";
        }
        void return_TVshow(const std::string& title){
            for(auto i = rented_TVshows.begin(); i!=rented_TVshows.end(); i++){
                if(to_lowercase(i->title) == to_lowercase(title)){
                    rented_TVshows.erase(i);
                    std::cout << "TV show " << title << " returned successfully.\n";
                    save_user_data();
                    return;
                }
            }
            std::cout << "TV show " << title << " not rented.\n";
        }
};

class admin : public user{
    public:
        admin(mediaLibrary& lib_ref) : lib(lib_ref){
            username = "admin";
        }
    private:
        mediaLibrary& lib;
    public:
        void add_movie(std::string title, std::string genre, double rating, int duration, double rent_cost, double purchase_cost){
            movie m;
            m.title=title;
            m.genre=genre;
            m.rating=rating;
            m.duration=duration;
            m.rent_cost=rent_cost;
            m.purchase_cost=purchase_cost;
            lib.add_movie(m);
        }
        void add_TVshow(std::string title, std::string genre, double rating, int seasons, int episodes_per_season, double season_rent_cost, double season_purchase_cost){
            TVshow t;
            t.title=title;
            t.genre=genre;
            t.rating=rating;
            t.seasons=seasons;
            t.episodes_per_season=episodes_per_season;
            t.season_rent_cost=season_rent_cost;
            t.season_purchase_cost=season_purchase_cost;
            lib.add_TVshow(t);
        }
        void remove_TVshow(const std::string& title){
            if(lib.remove_TVshow(title)){
                std::cout << "TV show " << title << " removed successfully.\n";
            }
            else{
                std::cout << "TV show " << title << " not found.\n";
            }
        }
        void remove_movie(const std::string& title){
            if(lib.remove_movie(title)){
                std::cout << "Movie " << title << " removed successfully.\n";
            }
            else{
                std::cout << "Movie " << title << " not found.\n";
            }
        }
        void admin_check_charges(user u){
            std::cout << u.username << " has Rs. " << u.charges_due << " due.\n";
        }
};

void logout(){
    std::cout << "Logged out successfully.\n";
}

bool userExists(std::string username){
    for(const auto& u : user_list){
        if(u.username == username){
            return true;
        }
    }
    return false;
}

void signup(){
    std::string username, password;
    std::cout << "Enter username: ";
    std::cin >> username;
    if(userExists(username)){
        std::cout << "Username already exists. Try again.\n";
        return;
    }
    std::cout << "Set password: ";
    std::cin >> password;
    user u;
    u.username = username;
    u.password = password;
    user_list.push_back(u);
    u.save_user_data(); // creates a file for that specific user

    std::ofstream fout("users.txt", std::ios::app); // users.txt stores all usernames
    fout << username << "\n";
    fout.close();

    std::cout << "Account created successfully.\n";
    return;
}

user* login(){
    std::string username, password;
    std::cout << "Enter username: ";
    std::cin >> username;
    std::cout << "Enter password: ";
    std::cin >> password;
    for(auto& u : user_list){
        if(u.username == username && u.password == password){
            std::cout << "Welcome " << username << "!\n";
            return &u;
        }
    }
    std::cout << "Invalid username or password!\n";
    return nullptr;
}

admin* admin_login(admin* a_ptr){
    std::string pass;
    std::cout << "Enter admin password: (password 1234) ";
    std::cin >> pass;
    if(pass == "1234"){
        std::cout << "Admin login successful.\n";
        return a_ptr;
    }
    else{
        std::cout << "Incorrect password!\n";
        return nullptr;
    }
}

void welcome(){
    static mediaLibrary library;
    static admin a(library);
    library.load_movie_data();
    library.load_TVshow_data();
    while(1){
        std::cout << "\nWelcome\n";
        std::cout << "Enter 1, 2 or 3. Any other input to exit.\n";
        std::cout << "1. User signup\n";
        std::cout << "2. User login\n";
        std::cout << "3. Admin login\n";
        std::string response;
        std::cin >> response;
        if(response == "1"){
            signup();
        }
        else if(response == "2"){
            user* u_ptr = login();
            user_page(u_ptr, library);
        }
        else if(response == "3"){
            admin* a_ptr = admin_login(&a);
            admin_page(a_ptr, library);
        }
        else{
            std::cout << "Goodbye!\n";
            library.save_movie_data();
            library.save_TVshow_data();
            return;
        }
    }
}

void user_page(user* u, mediaLibrary& library){
    if(u == nullptr) return;
    while(1){
        std::string response;
        std::cout << "Menu: Enter the number of the option you want to select.\n";
        std::cout << "1. Browse movies and TV shows\n";
        std::cout << "2. Search content by title\n";
        std::cout << "3. Search content by genre\n";
        std::cout << "4. Rent movie (for 3 days)\n";
        std::cout << "5. Rent TV show (for 3 days)\n";
        std::cout << "6. Purchase movie\n";
        std::cout << "7. Purchase TV show\n";
        std::cout << "8. View currently rented items\n";
        std::cout << "9. View purchased items\n";
        std::cout << "10. Return rented movie\n";
        std::cout << "11. Return rented TV show\n";
        std::cout << "12. Check total charges due\n";
        std::cout << "13. Logout\n";
        std::cin >> response;
        if(response == "1"){
            std::cout << "\nMovies\n";
            u->view_movies(library);
            std::cout << "\nTV shows\n";
            u->view_TVshows(library);
        }
        else if(response == "2"){
            std::string key;
            std::cout << "Enter keyword to search for: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, key);
            library.search_by_title(key);
        }
        else if(response == "3"){
            std::string key;
            std::cout << "Enter genre to search for: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, key);
            library.search_by_genre(key);
        }
        else if(response == "4"){
            std::string title;
            std::cout << "Enter title of movie to rent: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            u->rent_movie(title, library);
        }
        else if(response == "5"){
            std::string title;
            std::cout << "Enter title of TV show to rent: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            int num_seasons_rented;
            std::cout << "How many seasons do you want to rent? ";
            std::cin >> num_seasons_rented;
            u->rent_TVshow(title, num_seasons_rented, library);
        }
        else if(response == "6"){
            std::string title;
            std::cout << "Enter title of movie to purchase: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            u->purchase_movie(title, library);
        }
        else if(response == "7"){
            std::string title;
            std::cout << "Enter title of TV show to purchase: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            int num_seasons_purchased;
            std::cout << "How many seasons do you want to purchase? ";
            std::cin >> num_seasons_purchased;
            u->purchase_TVshow(title, num_seasons_purchased, library);
        }
        else if(response == "8"){
            u->view_rented_items();
        }
        else if(response == "9"){
            u->view_purchased_items();
        }
        else if(response == "10"){
            std::string title;
            std::cout << "Enter title of movie to return: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            u->return_movie(title);
        }
        else if(response == "11"){
            std::string title;
            std::cout << "Enter title of TV show to return: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            u->return_TVshow(title);
        }
        else if(response == "12"){
            u->check_charges();
        }
        else if(response == "13"){
            logout();
            return;
        }
        else{
            std::cout << "Invalid input!\n";
        }
    }
}

void admin_page(admin* a, mediaLibrary& library){
    if(a == nullptr) return;
    while(1){
        std::string response;
        std::cout << "Menu: Enter the number of the option you want to select.\n";
        std::cout << "1. Add new movie\n";
        std::cout << "2. Add new TV show\n";
        std::cout << "3. Remove movie\n";
        std::cout << "4. Remove TV show\n";
        std::cout << "5. Check charges due for a user\n";
        std::cout << "6. View all movies and TV shows\n";
        std::cout << "7. View all users (usernames only)\n";
        std::cout << "8. Logout\n";
        std::cin >> response;
        if(response == "1"){
            std::string title, genre;
            double rating, rent_cost, purchase_cost;
            int duration;
            std::cout << "Enter movie title: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            if(library.movie_exists(title)){
                continue;
            }
            std::cout << "Enter genre: ";
            std::getline(std::cin, genre);
            std::cout << "Enter rating (decimal number): ";
            std::cin >> rating;
            std::cout << "Enter movie duration (in minutes): ";
            std::cin >> duration;
            std::cout << "Enter rent cost: ";
            std::cin >> rent_cost;
            std::cout << "Enter purchase cost: ";
            std::cin >> purchase_cost;
            a->add_movie(title, genre, rating, duration, rent_cost, purchase_cost);
        }
        else if(response == "2"){
            std::string title, genre;
            int seasons, episodes_per_season;
            double rating, season_rent_cost, season_purchase_cost;
            std::cout << "Enter TV show title: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            if(library.TVshow_exists(title)){
                continue;
            }
            std::cout << "Enter genre: ";
            std::getline(std::cin, genre);
            std::cout << "Enter rating (decimal number): ";
            std::cin >> rating;
            std::cout << "Number of seasons: ";
            std::cin >> seasons;
            std::cout << "Number of episodes per season: ";
            std::cin >> episodes_per_season;
            std::cout << "Per season rent cost: ";
            std::cin >> season_rent_cost;
            std::cout << "Per season purchase cost: ";
            std::cin >> season_purchase_cost;
            a->add_TVshow(title, genre, rating, seasons, episodes_per_season, season_rent_cost, season_purchase_cost);
        }
        else if(response == "3"){
            std::string title;
            std::cout << "Enter title of movie to be removed: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            a->remove_movie(title);
        }
        else if(response == "4"){
            std::string title;
            std::cout << "Enter title of TV show to be removed: ";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, title);
            a->remove_TVshow(title);
        }
        else if(response == "5"){
            std::string username;
            std::cout << "Enter username of user: ";
            std::cin >> username;
            int flag = 1;
            for(const auto& u : user_list){
                if(u.username == username){
                    a->admin_check_charges(u);
                    flag = 0;
                    break;
                }
            }
            if(flag) std::cout << "User not found.\n";
        }
        else if(response == "6"){
            std::cout << "\nMovies\n";
            a->view_movies(library);
            std::cout << "\nTV shows\n";
            a->view_TVshows(library);
        }
        else if(response == "7"){
            if(user_list.empty()){
                std::cout << "No users\n";
            }
            else{    
                for(const auto& u : user_list){
                    std::cout << u.username << "\n";
                }
            }
        }
        else if(response == "8"){
            logout();
            return;
        }
        else{
            std::cout << "Invalid input!\n";
        }
    }
}

void load_all_users(){
    std::ifstream fin("users.txt");
    if(!fin) return;
    std::string username;
    while(std::getline(fin, username)){
        user u;
        u.username = username;
        u.load_user_data();
        user_list.push_back(u);
    }
    fin.close();
}

int main(){
    load_all_users();
    welcome();
    return 0;
}