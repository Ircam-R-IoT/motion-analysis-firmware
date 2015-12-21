
#define CONFIG_WEB_PAGE  0
#define PARAMS_WEB_PAGE   1


// HTML strings for the webserver 
const char HTTP_RESPONSE_0[] = {"HTTP/1.1 200 OK"};
const char HTTP_RESPONSE_1[] = {"Content-Type: text/html\n"};
const char HTTP_RESPONSE_2[] = {"<!DOCTYPE html>"};
          
// IRCAM logo (doesn't fit for now)
const char IRCAM_LOGO_0[] = {"<img src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAJEAAABVCAIAAADGw5HWAAAAB3R"};
const char IRCAM_LOGO_1[] = {"JTUUH3wQPDSEPlqcp7QAAAAlwSFlzAAAuGAAALhgBKqonIAAAAARnQU1BAACxjwv8YQUAABLjSURBVHja7V0LeBTV2T7nzOwlu9m"};
const char IRCAM_LOGO_2[] = {"QkARyA6O5cU1CLjxqQQUErWK1CgI+tFJqFapPay3aiqKPv/4V8ZdatT7+VUS8t4oo/L2gXEIVVMomMQiEhDsKCQkJ2WST3Z2dOf9"};
const char IRCAM_LOGO_3[] = {"3ZnYns5vdJWSTkGz2fSCZmXPd8875zvd95zsb7HbZUUSA05kwxurtYyseeWzF8ovdqT4BudgdiOKCEeVs8CHK2eADf7E70GsQnG2"};
const char IRCAM_LOGO_4[] = {"UUs0Dtyi4Lnan+gQ4YnSQoYOobBx8iHI2+BA5nIFx1trWpv5zuToXM4fDqU1SzThCyL791dm548G286utsbFx7ryFE/JLVjz+lCC"};
const char IRCAM_LOGO_5[] = {"4QzcN66jD6Xzm2ecn5Bf/8MZboGywnJIkaXui9vyNN9+bWFDy/AuvQJe0+Zubzy355a8hacu2z9VuR856FsKmXrR4ydtvv6UdYio"};
const char IRCAM_LOGO_6[] = {"6CB+jfSIK7cqF0RQvigFIEgW7j4rjxbmW1qTklIBdArVI2yVAdfXBCflF2nbzCwr3Vn2jzZM3Zsy+Kiu8KDHmYX4VLliw4K11r0X"};
const char IRCAM_LOGO_7[] = {"OPLsgaAlTASOoM8QGJAyxd8LsFPxJe/PtvwUjDAC10YA8e/Hww8v9CAMcrK7OG5vflTDAe++9t2XbF0ORM793X8WozJzQBc3mWG1"};
const char IRCAM_LOGO_8[] = {"ZTmdYvPhnoYuMTB0dInXlyqcDPj98+HCwItdff91Q5CwY6k6fPm+e/EmT1WuMz2/dNjU1wQTu3X4Oac6UhU0RX7ZWe4BUd8fSpUu"};
const char IRCAM_LOGO_9[] = {"0D/d9+61ywfH+hC1dsoSKHV2F4cGaY6H7ACpFebk1WFL1gQN+z4coZytXPgPDAXqHCICfQvs777yjzeB0sIeiJL34p+cC1tDSYtP"};
const char IRCAM_LOGO_10[] = {"ezp8//8UXnhNFCUo1nz2jTZo3f16wbkyZ8gNF98mfMMYvKTU1VUnKzhqdnDxCmxQ5vqsLwrIH7vN78va7f9Xe6g0xypABYmJMHR3"};
const char IRCAM_LOGO_11[] = {"tahKnM4qCw6DXnzhxHHlnVXpasjrB4hN81Ae7vSNYNz7//Au1le9PHkkfdZmadOrUKTXpyKH9lmFJatJQ5Gz31192fchxXLD8rS0"};
const char IRCAM_LOGO_12[] = {"N2lsgDH7qdHxaSuc40rDXLEnqbhVDkbPSyZerr3BvAdRJwuvltUbq6xVniK5nvQVQE95650Oww8B6w1iHMaf87NNGh+I86y0oNnj"};
const char IRCAM_LOGO_13[] = {"/txudZz3HmHEFF6XdKGc9BEyyrt6KhISEaddcPX78+D5tOiobewheb/Z74uywKconmNuYGPqu6eg86yF2f7VTe/vZp5tDWAu9i+g"};
const char IRCAM_LOGO_14[] = {"880ASpWBJFZXfiKKo3paWFINgPNPgs0l27cxZvW4/BEOUMw/mzLn1q6++DJhUOvlK7a3L0UoI2VNeqX1YX1+XNDzOc9PHun5UNnr"};
const char IRCAM_LOGO_15[] = {"wk4V3aG8xGFk6E6czrH7+Rb+cylZyyaR87cOUlFTVO9wdf384iM4zDxKHx/s9CbjNZrF4JtPUq6b6JYGtlpSU3NjYgPoY0XnWicT"};
const char IRCAM_LOGO_16[] = {"EpPPmsdnOKhdmU4Cd7n4gDEU506Lu1LHQGY4dqVXjXGEWFpeUBsxWWNi3tnaUs04ADYKzLTU1LWAqLFcZGT5JX+3c3jVbfHxCRfn"};
const char IRCAM_LOGO_17[] = {"uvu1nxMRdud1uQ0yc9omqfPuFwoHxy/OhFnJYyHbvqfz97x85efLE9GlXv/ra6yH0eKj89rlz9lgrpk6dsm7dOsgJ7PpZ3Jqe6GD"};
const char IRCAM_LOGO_18[] = {"hU5+DCqpadexCY4m7nG2cN24OXiZt0FHkcDZ0EJWNgw9RzgYfopwNPgxUm1qn5+S+uQU7Rjjs6iIKA44zKoprYzJLTPESYlxxmE9"};
const char IRCAM_LOGO_19[] = {"77dnht117sfs1gIDvvuceq9UafkXdxFVXXf3syv8KkWFzzJgUo55dERGJHPO3UjG77CNzYXaYTe++6xHHuxvgYlp9hWjpw/2tvgZ"};
const char IRCAM_LOGO_20[] = {"fc/Cgdc+efmsvNTU1RCrnJClGHuwbjnCncrKayivHmWLBPDk4c0FRw3/CbNrAEYNeR7EUZj0XHfyWT//eF/WCpUkIgdHftPGTH17"};
const char IRCAM_LOGO_21[] = {"fXcn2r1kLkinzzab/452Jl7Md+l2W8SaeI4KT6AyS4Ay3W5jiXg6evwjgy3Z80XDmTPgVabFlW9maNWuQ91zXB+s3IG/AZWp62pQ"};
const char IRCAM_LOGO_22[] = {"rLw9WkDS3AGMU0eSppYrvoMnZZuaHwROMwt2UokrV7HJwKzX8U//9dFlZWV9ULTjbBKddb/SJJpt9001Trnw/dEHtiN7cfEgeZuo"};
const char IRCAM_LOGO_23[] = {"W2pXnQqNtR1pRosUMFNe2ts6hx4kgIHlm79BnShgdHWZJGzMusbISIbGW4nlnK0G6/l2fk2Q0GDkCWuiO1EIi4Svb93Etzu3pk6B"};
const char IRCAM_LOGO_24[] = {"a17Sps/751kYyMiM2vqCpAmqzPvwn4eU1IE4lRFoKJ0zfsg6RgcI0ptQZnpXGK/tM+/btHTeu86CAKLjUg5dXXHHlrl07PAmgF7r"};
const char IRCAM_LOGO_25[] = {"FYHVtzZ+ddPIEZCpqOxzQxdf8wdYTv7gfOKCUeQVhIhOiG99SQUQKzZWbs0D22ezOeJNRYmKQwP+G4pKZW9buiB0br9NR+YWAnzw"};
const char IRCAM_LOGO_26[] = {"lE85Vci0ua2YJPPjO7hxlMsBE5CiaeK7qVWP25BgTyyp53qDjDuePOg5ebLK8I97R7qIyelD46mtmWK1Mf/GebfVYDvY2W0dHR/I"};
const char IRCAM_LOGO_27[] = {"I5iddv/6jURmp6954U0nKyMi45uopPewsRd/dvQxGFtqTOFgt2XiKklCWde30ms+ULBiROLPRjQlRpCBFw/5Tzj7nI8tsG/4Rd6i"};
const char IRCAM_LOGO_28[] = {"aSoT79b3tJqM3PxOao0xGJjU54qYEuJ9slB2ylLoJz1ERLi4x6KWjdeTSlJ50u9c5u+HGm8KUjSAD3S57sIja2267VXsLsrHHnGG"};
const char IRCAM_LOGO_29[] = {"9SUJuxlyH40ZnDdaZK2OzYSrHNNSreYCoeofruvaDSB9TGZsDqTqO8fKDh39ecewoqq2FLIXLFosWvaZeVir3jZdNN7Ot55O7dsn"};
const char IRCAM_LOGO_30[] = {"LJ6ovmDir7F3uiM1aPAVeyq0z75h5aFu/kHIekO3bt9Nu4NjRQ55B8QWwxevNCmGPrnjsvPVs2vhxz/sqcwI02DNGMucIzGymVWC"};
const char IRCAM_LOGO_31[] = {"jJkgNxt+WOoJNH6GDEuXEg1qc6YxMonbRHY/bbQphgHNV3yMMBfG0xx5g93kpjHMo1tpPYVXnRbf8IPn5BXv3VrEREztABmq3o44"};
const char IRCAM_LOGO_32[] = {"eO5GTOxYunE6X4OoAqRi6Ko7jdHy4i3mcU+QaHR5Fkko+WgvYX1Rrgfks1Z58XdaB+Btu7JrN2OBAZx0Kf7i3z9eGAz7YgfCuYHq"};
const char IRCAM_LOGO_33[] = {"g4NYbLQFTDQZ9dyoB2fjx+ve7kzM4SJrdbs0uVYJsYER9vmdA650M792w3rNUGRxYGsWBZB4QP9mlHlENKAMVwlaterY74jSIbNw"};
const char IRCAM_LOGO_34[] = {"YVn8ZJRIBxR5xekp0lFOue31c2F4z5nUSD01wlIe2+AHjP/GRjYlJSU1nWVyRctZYKwNXP//nhx56SPkwfd4pTQub9JemWyygK05"};
const char IRCAM_LOGO_35[] = {"uqVWSwAKrSTDffuSLPu4AN+bDV/kZJeqzcX3+sbsLf9k4evQlh2v3Qa95fWA9sPuyNCBmz77p44+CykaPlIMWGpvQMKaOEySCWYu"};
const char IRCAM_LOGO_36[] = {"9qoRsknWeo93+mydke4tM++OKC/nQuhCJIpVtCCopbi5Op99673L42KLBOGPl73pl0MMEX71/r3qj1+tHj85QbeFPPl6fl5vbu+2"};

const char IRCAM_LOGO_37[] = {"ZzOYQqfobZtBX1sIAvZ+cN4824LqmDHMCGGMSDJnLTvU8kZWMrLOtTAYcPjN87XroaaPDhbrBGSY8lSn/9Oafzdz+gRQkLCfr2qI"};

const char IRCAM_LOGO_38[] = {"ayozATbcunmM/vHHm/IwvmQ3amJPV93R0C3ze2LHa+4UL71SWtD6TgaIY3NV7zXMP7X3lDTCPcs3xFWZ5jDCb2Sct8ZNYJJvYRFE"};
const char IRCAM_LOGO_39[] = {"c0+fdVjlV9iDi+Ad/1Z2GJ616tPz9D6F00u6KKnPuhHOVAbPFZKXziHNT8TKEoA/psqYPwzGrYnO/naIIDYIxr/2n6iC4b/CHP6w"};
const char IRCAM_LOGO_40[] = {"K0RssCLkb3pSYR405lSm7QOX207OPe1xf09oPY45H8n4Klh0i5fbvi55Y6imOWBkaRHiLFq7CXu9vmzGTzceXhkWau2ENwhzx1Al"};
const char IRCAM_LOGO_41[] = {"9oKdKigYIYYj59bdv6c/2MjMzQ2cwTisoaaupuOvhbWv+bNAP+0Xl1sLsUWoqDNzE5oojT7z6yZPLjfywuyo+K8i71JtIzb9cDL/"};
const char IRCAM_LOGO_42[] = {"y4jzWiOWeO9mIa1SaRa4zXIOt+o+vcYrdZtTF3rsYcsSNy9P2QT+tuLitZsvMBVVb/u+yiZNvsW4W0QD6mtxofOPgQzTuavAhytn"};
const char IRCAM_LOGO_43[] = {"gQ5SzwYcBFyvXY1CC+Tob8myGsjMX4iVpSOgIt94LBBgk5JydKa8JsaKuiwpLEddoY6k6vZjQw9ivyNFB2D51bDZFIkbyfqdsCdQ"};
const char IRCAM_LOGO_44[] = {"IrjmtBzDpP3FSv7vy9KyfwEXq2hdG/niaXypxCBWpRcDkgfSUed9+1rMmIko2MrWeMlcJZbY2Adpy9MaNxv72XyhepMBmInuZJNn"};
const char IRCAM_LOGO_45[] = {"u7PnIR45sRIpNjfF3seYfVe1qP3V631U3EcmdaYqFKdhvFnHsqPQRy38DF/ETxwTMgCWO0RlGmGVEcabAznNoRKxpRE5pS7U1Ngt"};
const char IRCAM_LOGO_46[] = {"Y3P/K2ryfzwu/5u7AnJpsfjDUF0tTjMLc1Yko2egP7Pl4trLOGGSq03X9CwhasEhKTQaQtuw2uPcVUil/nmGEDGqdNJBYpBwO3Yo"};
const char IRCAM_LOGO_47[] = {"WkTXPqGdvGlYMJRrR+40d7AfHx3xpyTXIWy3w/4DesKDua0XJrEwsRJIkYq4qwTSpyYYpFglX2nrwPXNeLhbl7R4s/XZp0aPs+6R"};
const char IRCAM_LOGO_48[] = {"3zrnPvPXfMO6T2g5ZLbmYuUZprUuca9+PFB3kup/CRcrrz6f8eDqU/FvchDyeQE8kzJUeLMdEolQTwKIz/Dsmz8LJkXlY+t6FZrf"};
const char IRCAM_LOGO_49[] = {"uVXq1fURxghy6WdjMAjuqn3zZsfoVFGnzDHt48wkfBVVSlOAtrozNMSJlb4yxNtbpWDc835uFLTEcFWXCWD1wbTVn5yK3N4REIs+"};
const char IRCAM_LOGO_50[] = {"9rMwVLDsxJUTLY3MwFeXsOFfHc4JB7YUc9MCKtp2uy+WVAAVEoM6cAqhJjS6BX9+Yx8Vxnj4QSjJ4vC1xkrceHz0GY6XmMLSXgQk"};
const char IRCAM_LOGO_51[] = {"PJzK+/3Qnlj93/j0L15tzRVn02BGubK9jdhJGkyimvmEJDS7hQHuz4laGskB1pb1OtviYJlr7P/+L5BhyeUCxhGmF/TQi8hwi9JP"};
const char IRCAM_LOGO_52[] = {"pN3ftz+llq9kEolTAqNJeD0akdtPY9uKHEmbeZ4kjlfZGUd5NSZRg8oXalY00zoCXpCbbx5mTP0waV3/bIplCFDOj1NzepgzVFba"};
const char IRCAM_LOGO_53[] = {"qO11n9ukJ249mC41m6SJ0ZtuB+a7vau0t8ntPS23VkDl+9TPyG05rrSy8VZ6IUJiOev2lRa7G3LJ/MsMC6G9u7tqdk5V7qTzvJx/"};
const char IRCAM_LOGO_54[] = {"9dpGrvmBPmfzYQ9uG3z7AziFAQ001i1ync1Y9Lr8OIt5REeIjRhpn8J4Ox2R0Q1OWw8XmFaW26ddgQdTrdWyqsLnDtLbUwgK2X6o"};
const char IRCAM_LOGO_55[] = {"RPSwkGfPKQnIKObVBWyOn58N0lbCfoEIj514PV5airNCHbTzFklioBJ/qE7Hh2RLCWOTYbDPMmq4cATn2zf4QFUaWDuLRQiRlXSO"};
const char IRCAM_LOGO_56[] = {"Iu2T9X+JmTNYOXYiCIWIYKVuNgqb2cxxdhHEG48rVJJjnHNuprGy9VSlC5wlK7c+Q1QiTjUz2ycuNP2GYeo5ceHHB2pe2Qj+jmHp"};
const char IRCAM_LOGO_57[] = {"i/nsEn3LdsrYjjLOgUJS9zoEP74iun5yUl7NQs5pIneqpf3CUD9tEOZPQmdm3GvknN1Q484ypd7ycrS1dh6T78CgvDjlIxG47z1p"};
const char IRCAM_LOGO_58[] = {"IwSQTkY0Fb0rnXLKu7/0rj0oct7d7+OgJUQ4aapfUsCKsuL4BNTt2yldixHEWJO7qhOuMMgDvJhWffPmvaTVH2aFCRNzBxVHXatT"};
const char IRCAM_LOGO_59[] = {"BYoFBFH2dOP7U2k+2JRUrRzA4b+wQlYdWVoRQRnG+YpDtzBjf/P5m67ipnl7KuOKO+WCwQbWfWSZ89+qGqlvvlJ2RePz9i+V6sPJ"};
const char IRCAM_LOGO_60[] = {"6bEoqOvWX9RnWvcxXKeePHFDPgAZ45xfVef6G0nhBOLv8KTYeEj7icGD5D/YErIr6V605byNHXeqw1PDAinjiOT064fZblIxY3q9"};
const char IRCAM_LOGO_61[] = {"T5GfKk7+SGcRGSTy+ZBlHfQ6EZL/5NJUNhWTJ3fjgYzwV2aYpJqKLbdVW21oUp80oUTjz0OMEpquEuHB2cQYgvH6dABNNHG5yL5g"};
const char IRCAM_LOGO_62[] = {"rn15jqRKlTZjc4uw8buu3zChVeB8p9pffSQ6a8dJLbCdMNiw6OG7U/T/1FpREycOvJTO9lbBvz1D2iS6Zv8DTlNxTKtGcvZvVXkG"};
const char IRCAM_LOGO_63[] = {"Ck9OVNH+j1LLEeUggzA0HqRLhMu9eLHtjImifWv4w3mWAD3pSRmi1Szw28AaWRyXXr6D3lvKcMpZI+SZ3GFmO7Jp7n3kLi5Flh75"};
const char IRCAM_LOGO_64[] = {"d7cLps3iYiTMZPU4pNTNMNuKJkhXtHaid4xP1rAbla9+Zf5NTey7a23GdHafHEYMeaYx3SBJsbfBbD4IXNGL5VYgc+wyjUFSp0Fk"};
const char IRCAM_LOGO_65[] = {"CHRjwK+i99aoK2DcDRqpXEyNdWqJvP/wyMznKm03IHKQtuS6WISvADhEk6eO8rhN4AwgrG1GycYggylmPQPvV8eGHKGc9wkX9fsL"};
const char IRCAM_LOGO_66[] = {"IWc/6DxS32d3hV9Nj/D+VuT+1OqqeeQAAAABJRU5ErkJggg==\"/>"};

#define IRCAM_LOGO_SIZE    67
const char* pIRCAM_LOGO[] = {IRCAM_LOGO_0, IRCAM_LOGO_1, IRCAM_LOGO_2, IRCAM_LOGO_3, IRCAM_LOGO_4, IRCAM_LOGO_5, IRCAM_LOGO_6, IRCAM_LOGO_7, IRCAM_LOGO_8, IRCAM_LOGO_9,
                             IRCAM_LOGO_10, IRCAM_LOGO_11, IRCAM_LOGO_12, IRCAM_LOGO_13, IRCAM_LOGO_14, IRCAM_LOGO_15, IRCAM_LOGO_16, IRCAM_LOGO_17, IRCAM_LOGO_18, IRCAM_LOGO_19,
                             IRCAM_LOGO_20, IRCAM_LOGO_21, IRCAM_LOGO_22, IRCAM_LOGO_23, IRCAM_LOGO_24, IRCAM_LOGO_25, IRCAM_LOGO_26, IRCAM_LOGO_27, IRCAM_LOGO_28, IRCAM_LOGO_29,
                             IRCAM_LOGO_30, IRCAM_LOGO_31, IRCAM_LOGO_32, IRCAM_LOGO_33, IRCAM_LOGO_34, IRCAM_LOGO_35, IRCAM_LOGO_36, IRCAM_LOGO_37, IRCAM_LOGO_38, IRCAM_LOGO_39,
                             IRCAM_LOGO_40, IRCAM_LOGO_41, IRCAM_LOGO_42, IRCAM_LOGO_43, IRCAM_LOGO_44, IRCAM_LOGO_45, IRCAM_LOGO_46, IRCAM_LOGO_47, IRCAM_LOGO_48, IRCAM_LOGO_49,
                             IRCAM_LOGO_50, IRCAM_LOGO_51, IRCAM_LOGO_52, IRCAM_LOGO_53, IRCAM_LOGO_54, IRCAM_LOGO_55, IRCAM_LOGO_56, IRCAM_LOGO_57, IRCAM_LOGO_58, IRCAM_LOGO_59,                       
                             IRCAM_LOGO_60, IRCAM_LOGO_61, IRCAM_LOGO_62, IRCAM_LOGO_63, IRCAM_LOGO_64, IRCAM_LOGO_65, IRCAM_LOGO_66};
                             
const char HTML_HEADER_CSS_0[] = {"<html><head><title>R-IoT CC3200 WiFi Sensor Platform</title>\n<style>"};
const char HTML_HEADER_CSS_1[] = {"body {font-family:verdana; background-color:#FCFAF5; color:#7E7D7B}"};
const char HTML_HEADER_CSS_2[] = {"h1 {text-align:left; font-family:verdana;color:#7E7D7B}"};
const char HTML_HEADER_CSS_3[] = {"hr {color:#02C29D; background-color:#02C29D; border:0; height:8px}"};
const char HTML_HEADER_CSS_4[] = {"p {font-family:verdana;background-color:#F0EDE3;color:##666666}"};
const char HTML_HEADER_CSS_5[] = {"table {background-color:#F0EDE3;color:666666}\n</style></head>"};
const char HTML_HEADER_CSS_6[] = {"<html><head><title>R-IoT CC3200 WiFi Sensor Platform</title>"};
const char HTML_HEADER_CSS_7[] = {"<body>\n<h1><font color=\"#8BADFA\">{</font>SOUND MUSIC MOVEMENT<font color=\"#8BADFA\">}</font>"};
const char HTML_HEADER_CSS_8[] = {"INTERACTION<span style=\"float:right;\">"};

#define HTML_HEADER_CSS_SIZE    9                             

const char* pHTML_HEADER_CSS[] = {HTML_HEADER_CSS_0, HTML_HEADER_CSS_1, HTML_HEADER_CSS_2, HTML_HEADER_CSS_3, HTML_HEADER_CSS_4, HTML_HEADER_CSS_5, HTML_HEADER_CSS_6, HTML_HEADER_CSS_7, HTML_HEADER_CSS_8};
  
                             
                             
                             
                             
                             
                             
                             
