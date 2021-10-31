#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

int* END = new int(-1);

struct TNode {
    int l;
    int* r = END;
    std::unordered_map<char, TNode*> next;
    TNode* suffixLink = nullptr;
    TNode* backNode = nullptr;

    ~TNode(){
        for (auto& i: next){
            delete i.second;
        }
        if (r != END){
            delete r;
        }
    }
};

std::string PATTERN;
std::string TEXT;
TNode* ACTIVE_NODE = nullptr;
TNode* ACTIVE_EDGE = nullptr;
int ACTIVE_LENGTH = 0;
int REMAINDER = 0;
TNode* ROOT = new TNode();
TNode* PREVIOUS_NODE = ROOT;

void UpdateTree(int pos){
    ++(*END);
    ++REMAINDER;
    if (ACTIVE_EDGE == nullptr){
        auto next = ACTIVE_NODE->next.find(PATTERN[pos]);
        if (next != ACTIVE_NODE->next.end()){
            ++ACTIVE_LENGTH;
            ACTIVE_EDGE = next->second;
            if (ACTIVE_LENGTH == *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1){
                ACTIVE_NODE = ACTIVE_EDGE;
                ACTIVE_EDGE = nullptr;
                ACTIVE_LENGTH = 0;
            }
            return;
        }
    } else if (PATTERN[ACTIVE_EDGE->l + ACTIVE_LENGTH] == PATTERN[pos]){
        ++ACTIVE_LENGTH;
        if (ACTIVE_LENGTH == *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1){
            ACTIVE_NODE = ACTIVE_EDGE;
            ACTIVE_EDGE = nullptr;
            ACTIVE_LENGTH = 0;
        }
        return;
    }

    while (REMAINDER){
        if (ACTIVE_LENGTH != 0){
            TNode* newNode = new TNode();
            newNode->suffixLink = ROOT;
            newNode->l = pos;

            TNode* sideNode = new TNode();
            sideNode->l = ACTIVE_EDGE->l;
            sideNode->r = new int(sideNode->l + ACTIVE_LENGTH - 1);
            sideNode->suffixLink = ROOT;
            sideNode->backNode = ACTIVE_NODE;

            ACTIVE_NODE->next[PATTERN[ACTIVE_EDGE->l]] = sideNode;

            ACTIVE_EDGE->l = *sideNode->r + 1;
            ACTIVE_EDGE->backNode = sideNode;

            sideNode->next.insert({PATTERN[newNode->l], newNode});
            sideNode->next.insert({PATTERN[ACTIVE_EDGE->l], ACTIVE_EDGE});

            newNode->backNode = sideNode;

            ACTIVE_EDGE = sideNode;
            if (PREVIOUS_NODE != ROOT ){
                PREVIOUS_NODE->suffixLink = ACTIVE_EDGE;
            }
        } else {
            auto next = ACTIVE_NODE->next.find(PATTERN[pos]);
            if (next == ACTIVE_NODE->next.end()){
                TNode* newNode = new TNode();
                newNode->suffixLink = ROOT;
                newNode->l = pos;
                newNode->backNode = ACTIVE_NODE;
                ACTIVE_NODE->next.insert({PATTERN[newNode->l], newNode});
            } else {
                ACTIVE_EDGE = next->second;
                ++ACTIVE_LENGTH;
                if (ACTIVE_LENGTH == *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1) {
                    ACTIVE_NODE = ACTIVE_EDGE;
                    ACTIVE_LENGTH = 0;
                    ACTIVE_EDGE = nullptr;
                }
                PREVIOUS_NODE = ROOT;
                return;
            }
        }

        --REMAINDER;

        if (REMAINDER) {
            if (ACTIVE_LENGTH != 0) {
                PREVIOUS_NODE = ACTIVE_EDGE;
            } else {
                PREVIOUS_NODE = ROOT;
            }

            while (ACTIVE_NODE->suffixLink == ROOT && ACTIVE_NODE != ROOT){
                ACTIVE_LENGTH += *ACTIVE_NODE->r - ACTIVE_NODE->l + 1;
                ACTIVE_NODE = ACTIVE_NODE->backNode;
            }
            ACTIVE_NODE = ACTIVE_NODE->suffixLink;

            int rest;
            if (ACTIVE_NODE == ROOT){
                rest = REMAINDER - 1;
            } else {
                rest = ACTIVE_LENGTH;
            }

            ACTIVE_EDGE = nullptr;
            ACTIVE_LENGTH = 0;

            while (rest > 0) {
                auto next = ACTIVE_NODE->next.find(PATTERN[pos - rest]);
                ACTIVE_EDGE = next->second;
                if (*ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1 <= rest){
                    rest -= *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1;
                    ACTIVE_NODE = ACTIVE_EDGE;
                    ACTIVE_LENGTH = 0;
                    ACTIVE_EDGE = nullptr;
                } else {
                    ACTIVE_LENGTH = rest;
                    rest = 0;
                }
            }
        }
    }

    ACTIVE_EDGE = nullptr;
    ACTIVE_LENGTH = 0;
    ACTIVE_NODE = ROOT;
    PREVIOUS_NODE = ROOT;
}

void Build(){
    ROOT->l = -1;
    ROOT->r = new int(-1);
    ROOT->suffixLink = ROOT;
    ACTIVE_NODE = ROOT;
    for (int i = 0; i < PATTERN.length(); ++i){
        UpdateTree(i);
    }
}

void removeFirst(int pos){
    while (ACTIVE_NODE->suffixLink == ROOT && ACTIVE_NODE != ROOT){
        ACTIVE_LENGTH += *ACTIVE_NODE->r - ACTIVE_NODE->l + 1;
        ACTIVE_NODE = ACTIVE_NODE->backNode;
    }
    ACTIVE_NODE = ACTIVE_NODE->suffixLink;

    int cnt;
    if (ACTIVE_NODE == ROOT){
        cnt = REMAINDER - 1;
    } else {
        cnt = ACTIVE_LENGTH;
    }

    ACTIVE_EDGE = nullptr;
    ACTIVE_LENGTH = 0;

    while (cnt > 0) {
        auto next = ACTIVE_NODE->next.find(TEXT[pos - cnt]);
        ACTIVE_EDGE = next->second;
        if (*ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1 <= cnt){
            cnt -= *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1;
            ACTIVE_NODE = ACTIVE_EDGE;
            ACTIVE_LENGTH = 0;
            ACTIVE_EDGE = nullptr;
        } else {
            ACTIVE_LENGTH = cnt;
            cnt = 0;
        }
    }
}

int main(){
    std::cin >> PATTERN;
    PATTERN += '$';
    Build();

    std::cin >> TEXT;
    TEXT += '$';
    std::vector<int> matchStatistic(TEXT.length());

    for (int i = 0; i < TEXT.length(); ++i){
        ++REMAINDER;
        if (ACTIVE_EDGE == nullptr){
            auto next = ACTIVE_NODE->next.find(TEXT[i]);
            if (next != ACTIVE_NODE->next.end()){
                ++ACTIVE_LENGTH;
                ACTIVE_EDGE = next->second;
                if (ACTIVE_LENGTH == *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1){
                    ACTIVE_NODE = ACTIVE_EDGE;
                    ACTIVE_EDGE = nullptr;
                    ACTIVE_LENGTH = 0;
                }
            } else {
                --REMAINDER;
                matchStatistic[i - REMAINDER] = REMAINDER;
                if (REMAINDER == PATTERN.length() - 1){
                    std::cout << i - REMAINDER + 1 << '\n';
                }
                removeFirst(i);
                if (REMAINDER){
                    --REMAINDER;
                    --i;
                }
            }
        } else if (PATTERN[ACTIVE_EDGE->l + ACTIVE_LENGTH] == TEXT[i]){
            ++ACTIVE_LENGTH;
            if (ACTIVE_LENGTH == *ACTIVE_EDGE->r - ACTIVE_EDGE->l + 1){
                ACTIVE_NODE = ACTIVE_EDGE;
                ACTIVE_EDGE = nullptr;
                ACTIVE_LENGTH = 0;
            }
        } else {
            --REMAINDER;
            matchStatistic[i - REMAINDER] = REMAINDER;
            if (REMAINDER == PATTERN.length() - 1){
                std::cout << i - REMAINDER + 1 << '\n';
            }
            removeFirst(i);
            if (REMAINDER){
                --REMAINDER;
                --i;
            }
        }
    }

    while (REMAINDER){
        matchStatistic[matchStatistic.size() - REMAINDER] = REMAINDER - 1;
        if (REMAINDER - 1 == PATTERN.length() - 1){
            std::cout << matchStatistic.size() - REMAINDER + 1 << '\n';
        }
        --REMAINDER;
    }

    delete ROOT;
    delete END;

    return 0;
}