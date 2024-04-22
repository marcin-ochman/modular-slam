#include "modular_slam/distributed_cv_feature.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>
#ifdef USE_SSE_ORB
#ifdef _MSC_VER
#include <intrin.h>
#else
#include <x86intrin.h>
#endif
#endif // USE_SSE_ORB

namespace mslam
{

// NOTE: Part of this code is modified version of orb extractor found in stella vslam

// (X, Y) x 2 points x 256 bits = 1024
static constexpr unsigned int orb_point_pairs_size = 256 * 4;

// quoted from
// https://github.com/opencv/opencv/blob/50bec53afcf010e425b3f015c71297d46ef78903/modules/features2d/src/orb.cpp#L375
alignas(16) static constexpr float orb_point_pairs[orb_point_pairs_size] = {
    8,   -3,  9,   5,   /* mean (0), correlation (0) */
    4,   2,   7,   -12, /* mean (1.12461e-05), correlation (0.0437584) */
    -11, 9,   -8,  2,   /* mean (3.37382e-05), correlation (0.0617409) */
    7,   -12, 12,  -13, /* mean (5.62303e-05), correlation (0.0636977) */
    2,   -13, 2,   12,  /* mean (0.000134953), correlation (0.085099) */
    1,   -7,  1,   6,   /* mean (0.000528565), correlation (0.0857175) */
    -2,  -10, -2,  -4,  /* mean (0.0188821), correlation (0.0985774) */
    -13, -13, -11, -8,  /* mean (0.0363135), correlation (0.0899616) */
    -13, -3,  -12, -9,  /* mean (0.121806), correlation (0.099849) */
    10,  4,   11,  9,   /* mean (0.122065), correlation (0.093285) */
    -13, -8,  -8,  -9,  /* mean (0.162787), correlation (0.0942748) */
    -11, 7,   -9,  12,  /* mean (0.21561), correlation (0.0974438) */
    7,   7,   12,  6,   /* mean (0.160583), correlation (0.130064) */
    -4,  -5,  -3,  0,   /* mean (0.228171), correlation (0.132998) */
    -13, 2,   -12, -3,  /* mean (0.00997526), correlation (0.145926) */
    -9,  0,   -7,  5,   /* mean (0.198234), correlation (0.143636) */
    12,  -6,  12,  -1,  /* mean (0.0676226), correlation (0.16689) */
    -3,  6,   -2,  12,  /* mean (0.166847), correlation (0.171682) */
    -6,  -13, -4,  -8,  /* mean (0.101215), correlation (0.179716) */
    11,  -13, 12,  -8,  /* mean (0.200641), correlation (0.192279) */
    4,   7,   5,   1,   /* mean (0.205106), correlation (0.186848) */
    5,   -3,  10,  -3,  /* mean (0.234908), correlation (0.192319) */
    3,   -7,  6,   12,  /* mean (0.0709964), correlation (0.210872) */
    -8,  -7,  -6,  -2,  /* mean (0.0939834), correlation (0.212589) */
    -2,  11,  -1,  -10, /* mean (0.127778), correlation (0.20866) */
    -13, 12,  -8,  10,  /* mean (0.14783), correlation (0.206356) */
    -7,  3,   -5,  -3,  /* mean (0.182141), correlation (0.198942) */
    -4,  2,   -3,  7,   /* mean (0.188237), correlation (0.21384) */
    -10, -12, -6,  11,  /* mean (0.14865), correlation (0.23571) */
    5,   -12, 6,   -7,  /* mean (0.222312), correlation (0.23324) */
    5,   -6,  7,   -1,  /* mean (0.229082), correlation (0.23389) */
    1,   0,   4,   -5,  /* mean (0.241577), correlation (0.215286) */
    9,   11,  11,  -13, /* mean (0.00338507), correlation (0.251373) */
    4,   7,   4,   12,  /* mean (0.131005), correlation (0.257622) */
    2,   -1,  4,   4,   /* mean (0.152755), correlation (0.255205) */
    -4,  -12, -2,  7,   /* mean (0.182771), correlation (0.244867) */
    -8,  -5,  -7,  -10, /* mean (0.186898), correlation (0.23901) */
    4,   11,  9,   12,  /* mean (0.226226), correlation (0.258255) */
    0,   -8,  1,   -13, /* mean (0.0897886), correlation (0.274827) */
    -13, -2,  -8,  2,   /* mean (0.148774), correlation (0.28065) */
    -3,  -2,  -2,  3,   /* mean (0.153048), correlation (0.283063) */
    -6,  9,   -4,  -9,  /* mean (0.169523), correlation (0.278248) */
    8,   12,  10,  7,   /* mean (0.225337), correlation (0.282851) */
    0,   9,   1,   3,   /* mean (0.226687), correlation (0.278734) */
    7,   -5,  11,  -10, /* mean (0.00693882), correlation (0.305161) */
    -13, -6,  -11, 0,   /* mean (0.0227283), correlation (0.300181) */
    10,  7,   12,  1,   /* mean (0.125517), correlation (0.31089) */
    -6,  -3,  -6,  12,  /* mean (0.131748), correlation (0.312779) */
    10,  -9,  12,  -4,  /* mean (0.144827), correlation (0.292797) */
    -13, 8,   -8,  -12, /* mean (0.149202), correlation (0.308918) */
    -13, 0,   -8,  -4,  /* mean (0.160909), correlation (0.310013) */
    3,   3,   7,   8,   /* mean (0.177755), correlation (0.309394) */
    5,   7,   10,  -7,  /* mean (0.212337), correlation (0.310315) */
    -1,  7,   1,   -12, /* mean (0.214429), correlation (0.311933) */
    3,   -10, 5,   6,   /* mean (0.235807), correlation (0.313104) */
    2,   -4,  3,   -10, /* mean (0.00494827), correlation (0.344948) */
    -13, 0,   -13, 5,   /* mean (0.0549145), correlation (0.344675) */
    -13, -7,  -12, 12,  /* mean (0.103385), correlation (0.342715) */
    -13, 3,   -11, 8,   /* mean (0.134222), correlation (0.322922) */
    -7,  12,  -4,  7,   /* mean (0.153284), correlation (0.337061) */
    6,   -10, 12,  8,   /* mean (0.154881), correlation (0.329257) */
    -9,  -1,  -7,  -6,  /* mean (0.200967), correlation (0.33312) */
    -2,  -5,  0,   12,  /* mean (0.201518), correlation (0.340635) */
    -12, 5,   -7,  5,   /* mean (0.207805), correlation (0.335631) */
    3,   -10, 8,   -13, /* mean (0.224438), correlation (0.34504) */
    -7,  -7,  -4,  5,   /* mean (0.239361), correlation (0.338053) */
    -3,  -2,  -1,  -7,  /* mean (0.240744), correlation (0.344322) */
    2,   9,   5,   -11, /* mean (0.242949), correlation (0.34145) */
    -11, -13, -5,  -13, /* mean (0.244028), correlation (0.336861) */
    -1,  6,   0,   -1,  /* mean (0.247571), correlation (0.343684) */
    5,   -3,  5,   2,   /* mean (0.000697256), correlation (0.357265) */
    -4,  -13, -4,  12,  /* mean (0.00213675), correlation (0.373827) */
    -9,  -6,  -9,  6,   /* mean (0.0126856), correlation (0.373938) */
    -12, -10, -8,  -4,  /* mean (0.0152497), correlation (0.364237) */
    10,  2,   12,  -3,  /* mean (0.0299933), correlation (0.345292) */
    7,   12,  12,  12,  /* mean (0.0307242), correlation (0.366299) */
    -7,  -13, -6,  5,   /* mean (0.0534975), correlation (0.368357) */
    -4,  9,   -3,  4,   /* mean (0.099865), correlation (0.372276) */
    7,   -1,  12,  2,   /* mean (0.117083), correlation (0.364529) */
    -7,  6,   -5,  1,   /* mean (0.126125), correlation (0.369606) */
    -13, 11,  -12, 5,   /* mean (0.130364), correlation (0.358502) */
    -3,  7,   -2,  -6,  /* mean (0.131691), correlation (0.375531) */
    7,   -8,  12,  -7,  /* mean (0.160166), correlation (0.379508) */
    -13, -7,  -11, -12, /* mean (0.167848), correlation (0.353343) */
    1,   -3,  12,  12,  /* mean (0.183378), correlation (0.371916) */
    2,   -6,  3,   0,   /* mean (0.228711), correlation (0.371761) */
    -4,  3,   -2,  -13, /* mean (0.247211), correlation (0.364063) */
    -1,  -13, 1,   9,   /* mean (0.249325), correlation (0.378139) */
    7,   1,   8,   -6,  /* mean (0.000652272), correlation (0.411682) */
    1,   -1,  3,   12,  /* mean (0.00248538), correlation (0.392988) */
    9,   1,   12,  6,   /* mean (0.0206815), correlation (0.386106) */
    -1,  -9,  -1,  3,   /* mean (0.0364485), correlation (0.410752) */
    -13, -13, -10, 5,   /* mean (0.0376068), correlation (0.398374) */
    7,   7,   10,  12,  /* mean (0.0424202), correlation (0.405663) */
    12,  -5,  12,  9,   /* mean (0.0942645), correlation (0.410422) */
    6,   3,   7,   11,  /* mean (0.1074), correlation (0.413224) */
    5,   -13, 6,   10,  /* mean (0.109256), correlation (0.408646) */
    2,   -12, 2,   3,   /* mean (0.131691), correlation (0.416076) */
    3,   8,   4,   -6,  /* mean (0.165081), correlation (0.417569) */
    2,   6,   12,  -13, /* mean (0.171874), correlation (0.408471) */
    9,   -12, 10,  3,   /* mean (0.175146), correlation (0.41296) */
    -8,  4,   -7,  9,   /* mean (0.183682), correlation (0.402956) */
    -11, 12,  -4,  -6,  /* mean (0.184672), correlation (0.416125) */
    1,   12,  2,   -8,  /* mean (0.191487), correlation (0.386696) */
    6,   -9,  7,   -4,  /* mean (0.192668), correlation (0.394771) */
    2,   3,   3,   -2,  /* mean (0.200157), correlation (0.408303) */
    6,   3,   11,  0,   /* mean (0.204588), correlation (0.411762) */
    3,   -3,  8,   -8,  /* mean (0.205904), correlation (0.416294) */
    7,   8,   9,   3,   /* mean (0.213237), correlation (0.409306) */
    -11, -5,  -6,  -4,  /* mean (0.243444), correlation (0.395069) */
    -10, 11,  -5,  10,  /* mean (0.247672), correlation (0.413392) */
    -5,  -8,  -3,  12,  /* mean (0.24774), correlation (0.411416) */
    -10, 5,   -9,  0,   /* mean (0.00213675), correlation (0.454003) */
    8,   -1,  12,  -6,  /* mean (0.0293635), correlation (0.455368) */
    4,   -6,  6,   -11, /* mean (0.0404971), correlation (0.457393) */
    -10, 12,  -8,  7,   /* mean (0.0481107), correlation (0.448364) */
    4,   -2,  6,   7,   /* mean (0.050641), correlation (0.455019) */
    -2,  0,   -2,  12,  /* mean (0.0525978), correlation (0.44338) */
    -5,  -8,  -5,  2,   /* mean (0.0629667), correlation (0.457096) */
    7,   -6,  10,  12,  /* mean (0.0653846), correlation (0.445623) */
    -9,  -13, -8,  -8,  /* mean (0.0858749), correlation (0.449789) */
    -5,  -13, -5,  -2,  /* mean (0.122402), correlation (0.450201) */
    8,   -8,  9,   -13, /* mean (0.125416), correlation (0.453224) */
    -9,  -11, -9,  0,   /* mean (0.130128), correlation (0.458724) */
    1,   -8,  1,   -2,  /* mean (0.132467), correlation (0.440133) */
    7,   -4,  9,   1,   /* mean (0.132692), correlation (0.454) */
    -2,  1,   -1,  -4,  /* mean (0.135695), correlation (0.455739) */
    11,  -6,  12,  -11, /* mean (0.142904), correlation (0.446114) */
    -12, -9,  -6,  4,   /* mean (0.146165), correlation (0.451473) */
    3,   7,   7,   12,  /* mean (0.147627), correlation (0.456643) */
    5,   5,   10,  8,   /* mean (0.152901), correlation (0.455036) */
    0,   -4,  2,   8,   /* mean (0.167083), correlation (0.459315) */
    -9,  12,  -5,  -13, /* mean (0.173234), correlation (0.454706) */
    0,   7,   2,   12,  /* mean (0.18312), correlation (0.433855) */
    -1,  2,   1,   7,   /* mean (0.185504), correlation (0.443838) */
    5,   11,  7,   -9,  /* mean (0.185706), correlation (0.451123) */
    3,   5,   6,   -8,  /* mean (0.188968), correlation (0.455808) */
    -13, -4,  -8,  9,   /* mean (0.191667), correlation (0.459128) */
    -5,  9,   -3,  -3,  /* mean (0.193196), correlation (0.458364) */
    -4,  -7,  -3,  -12, /* mean (0.196536), correlation (0.455782) */
    6,   5,   8,   0,   /* mean (0.1972), correlation (0.450481) */
    -7,  6,   -6,  12,  /* mean (0.199438), correlation (0.458156) */
    -13, 6,   -5,  -2,  /* mean (0.211224), correlation (0.449548) */
    1,   -10, 3,   10,  /* mean (0.211718), correlation (0.440606) */
    4,   1,   8,   -4,  /* mean (0.213034), correlation (0.443177) */
    -2,  -2,  2,   -13, /* mean (0.234334), correlation (0.455304) */
    2,   -12, 12,  12,  /* mean (0.235684), correlation (0.443436) */
    -2,  -13, 0,   -6,  /* mean (0.237674), correlation (0.452525) */
    4,   1,   9,   3,   /* mean (0.23962), correlation (0.444824) */
    -6,  -10, -3,  -5,  /* mean (0.248459), correlation (0.439621) */
    -3,  -13, -1,  1,   /* mean (0.249505), correlation (0.456666) */
    7,   5,   12,  -11, /* mean (0.00119208), correlation (0.495466) */
    4,   -2,  5,   -7,  /* mean (0.00372245), correlation (0.484214) */
    -13, 9,   -9,  -5,  /* mean (0.00741116), correlation (0.499854) */
    7,   1,   8,   6,   /* mean (0.0208952), correlation (0.499773) */
    7,   -8,  7,   6,   /* mean (0.0220085), correlation (0.501609) */
    -7,  -4,  -7,  1,   /* mean (0.0233806), correlation (0.496568) */
    -8,  11,  -7,  -8,  /* mean (0.0236505), correlation (0.489719) */
    -13, 6,   -12, -8,  /* mean (0.0268781), correlation (0.503487) */
    2,   4,   3,   9,   /* mean (0.0323324), correlation (0.501938) */
    10,  -5,  12,  3,   /* mean (0.0399235), correlation (0.494029) */
    -6,  -5,  -6,  7,   /* mean (0.0420153), correlation (0.486579) */
    8,   -3,  9,   -8,  /* mean (0.0548021), correlation (0.484237) */
    2,   -12, 2,   8,   /* mean (0.0616622), correlation (0.496642) */
    -11, -2,  -10, 3,   /* mean (0.0627755), correlation (0.498563) */
    -12, -13, -7,  -9,  /* mean (0.0829622), correlation (0.495491) */
    -11, 0,   -10, -5,  /* mean (0.0843342), correlation (0.487146) */
    5,   -3,  11,  8,   /* mean (0.0929937), correlation (0.502315) */
    -2,  -13, -1,  12,  /* mean (0.113327), correlation (0.48941) */
    -1,  -8,  0,   9,   /* mean (0.132119), correlation (0.467268) */
    -13, -11, -12, -5,  /* mean (0.136269), correlation (0.498771) */
    -10, -2,  -10, 11,  /* mean (0.142173), correlation (0.498714) */
    -3,  9,   -2,  -13, /* mean (0.144141), correlation (0.491973) */
    2,   -3,  3,   2,   /* mean (0.14892), correlation (0.500782) */
    -9,  -13, -4,  0,   /* mean (0.150371), correlation (0.498211) */
    -4,  6,   -3,  -10, /* mean (0.152159), correlation (0.495547) */
    -4,  12,  -2,  -7,  /* mean (0.156152), correlation (0.496925) */
    -6,  -11, -4,  9,   /* mean (0.15749), correlation (0.499222) */
    6,   -3,  6,   11,  /* mean (0.159211), correlation (0.503821) */
    -13, 11,  -5,  5,   /* mean (0.162427), correlation (0.501907) */
    11,  11,  12,  6,   /* mean (0.16652), correlation (0.497632) */
    7,   -5,  12,  -2,  /* mean (0.169141), correlation (0.484474) */
    -1,  12,  0,   7,   /* mean (0.169456), correlation (0.495339) */
    -4,  -8,  -3,  -2,  /* mean (0.171457), correlation (0.487251) */
    -7,  1,   -6,  7,   /* mean (0.175), correlation (0.500024) */
    -13, -12, -8,  -13, /* mean (0.175866), correlation (0.497523) */
    -7,  -2,  -6,  -8,  /* mean (0.178273), correlation (0.501854) */
    -8,  5,   -6,  -9,  /* mean (0.181107), correlation (0.494888) */
    -5,  -1,  -4,  5,   /* mean (0.190227), correlation (0.482557) */
    -13, 7,   -8,  10,  /* mean (0.196739), correlation (0.496503) */
    1,   5,   5,   -13, /* mean (0.19973), correlation (0.499759) */
    1,   0,   10,  -13, /* mean (0.204465), correlation (0.49873) */
    9,   12,  10,  -1,  /* mean (0.209334), correlation (0.49063) */
    5,   -8,  10,  -9,  /* mean (0.211134), correlation (0.503011) */
    -1,  11,  1,   -13, /* mean (0.212), correlation (0.499414) */
    -9,  -3,  -6,  2,   /* mean (0.212168), correlation (0.480739) */
    -1,  -10, 1,   12,  /* mean (0.212731), correlation (0.502523) */
    -13, 1,   -8,  -10, /* mean (0.21327), correlation (0.489786) */
    8,   -11, 10,  -6,  /* mean (0.214159), correlation (0.488246) */
    2,   -13, 3,   -6,  /* mean (0.216993), correlation (0.50287) */
    7,   -13, 12,  -9,  /* mean (0.223639), correlation (0.470502) */
    -10, -10, -5,  -7,  /* mean (0.224089), correlation (0.500852) */
    -10, -8,  -8,  -13, /* mean (0.228666), correlation (0.502629) */
    4,   -6,  8,   5,   /* mean (0.22906), correlation (0.498305) */
    3,   12,  8,   -13, /* mean (0.233378), correlation (0.503825) */
    -4,  2,   -3,  -3,  /* mean (0.234323), correlation (0.476692) */
    5,   -13, 10,  -12, /* mean (0.236392), correlation (0.475462) */
    4,   -13, 5,   -1,  /* mean (0.236842), correlation (0.504132) */
    -9,  9,   -4,  3,   /* mean (0.236977), correlation (0.497739) */
    0,   3,   3,   -9,  /* mean (0.24314), correlation (0.499398) */
    -12, 1,   -6,  1,   /* mean (0.243297), correlation (0.489447) */
    3,   2,   4,   -8,  /* mean (0.00155196), correlation (0.553496) */
    -10, -10, -10, 9,   /* mean (0.00239541), correlation (0.54297) */
    8,   -13, 12,  12,  /* mean (0.0034413), correlation (0.544361) */
    -8,  -12, -6,  -5,  /* mean (0.003565), correlation (0.551225) */
    2,   2,   3,   7,   /* mean (0.00835583), correlation (0.55285) */
    10,  6,   11,  -8,  /* mean (0.00885065), correlation (0.540913) */
    6,   8,   8,   -12, /* mean (0.0101552), correlation (0.551085) */
    -7,  10,  -6,  5,   /* mean (0.0102227), correlation (0.533635) */
    -3,  -9,  -3,  9,   /* mean (0.0110211), correlation (0.543121) */
    -1,  -13, -1,  5,   /* mean (0.0113473), correlation (0.550173) */
    -3,  -7,  -3,  4,   /* mean (0.0140913), correlation (0.554774) */
    -8,  -2,  -8,  3,   /* mean (0.017049), correlation (0.55461) */
    4,   2,   12,  12,  /* mean (0.01778), correlation (0.546921) */
    2,   -5,  3,   11,  /* mean (0.0224022), correlation (0.549667) */
    6,   -9,  11,  -13, /* mean (0.029161), correlation (0.546295) */
    3,   -1,  7,   12,  /* mean (0.0303081), correlation (0.548599) */
    11,  -1,  12,  4,   /* mean (0.0355151), correlation (0.523943) */
    -3,  0,   -3,  6,   /* mean (0.0417904), correlation (0.543395) */
    4,   -11, 4,   12,  /* mean (0.0487292), correlation (0.542818) */
    2,   -4,  2,   1,   /* mean (0.0575124), correlation (0.554888) */
    -10, -6,  -8,  1,   /* mean (0.0594242), correlation (0.544026) */
    -13, 7,   -11, 1,   /* mean (0.0597391), correlation (0.550524) */
    -13, 12,  -11, -13, /* mean (0.0608974), correlation (0.55383) */
    6,   0,   11,  -13, /* mean (0.065126), correlation (0.552006) */
    0,   -1,  1,   4,   /* mean (0.074224), correlation (0.546372) */
    -13, 3,   -9,  -2,  /* mean (0.0808592), correlation (0.554875) */
    -9,  8,   -6,  -3,  /* mean (0.0883378), correlation (0.551178) */
    -13, -6,  -8,  -2,  /* mean (0.0901035), correlation (0.548446) */
    5,   -9,  8,   10,  /* mean (0.0949843), correlation (0.554694) */
    2,   7,   3,   -9,  /* mean (0.0994152), correlation (0.550979) */
    -1,  -6,  -1,  -1,  /* mean (0.10045), correlation (0.552714) */
    9,   5,   11,  -2,  /* mean (0.100686), correlation (0.552594) */
    11,  -3,  12,  -8,  /* mean (0.101091), correlation (0.532394) */
    3,   0,   3,   5,   /* mean (0.101147), correlation (0.525576) */
    -1,  4,   0,   10,  /* mean (0.105263), correlation (0.531498) */
    3,   -6,  4,   5,   /* mean (0.110785), correlation (0.540491) */
    -13, 0,   -10, 5,   /* mean (0.112798), correlation (0.536582) */
    5,   8,   12,  11,  /* mean (0.114181), correlation (0.555793) */
    8,   9,   9,   -6,  /* mean (0.117431), correlation (0.553763) */
    7,   -4,  8,   -12, /* mean (0.118522), correlation (0.553452) */
    -10, 4,   -10, 9,   /* mean (0.12094), correlation (0.554785) */
    7,   3,   12,  4,   /* mean (0.122582), correlation (0.555825) */
    9,   -7,  10,  -2,  /* mean (0.124978), correlation (0.549846) */
    7,   0,   12,  -2,  /* mean (0.127002), correlation (0.537452) */
    -1,  -6,  0,   -11, /* mean (0.127148), correlation (0.547401) */
};

class orb_extractor_node
{
  public:
    //! Constructor
    orb_extractor_node() = default;

    //! Divide node to four child nodes
    std::array<orb_extractor_node, 4> divide_node();

    //! Size of area
    unsigned int size() const { return (pt_end_.x - pt_begin_.x) * (pt_end_.y - pt_begin_.y); }

    //! Keypoints which distributed into this node
    std::vector<cv::KeyPoint> keypts_;

    //! Begin and end of the allocated area on the image
    cv::Point2i pt_begin_, pt_end_;

    //! A iterator pointing to self, used for removal on list
    std::list<orb_extractor_node>::iterator iter_;
};

std::array<orb_extractor_node, 4> orb_extractor_node::divide_node()
{
    // Half width/height of the allocated patch area
    const unsigned int half_x = cvCeil((pt_end_.x - pt_begin_.x) / 2.0);
    const unsigned int half_y = cvCeil((pt_end_.y - pt_begin_.y) / 2.0);

    // Four new child nodes
    std::array<orb_extractor_node, 4> child_nodes;

    // A position of center top, left center, center, right center, and center bottom
    // These positions are used to determine new split areas
    const auto pt_top = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y);
    const auto pt_left = cv::Point2i(pt_begin_.x, pt_begin_.y + half_y);
    const auto pt_center = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y + half_y);
    const auto pt_right = cv::Point2i(pt_end_.x, pt_begin_.y + half_y);
    const auto pt_bottom = cv::Point2i(pt_begin_.x + half_x, pt_end_.y);

    // Assign new patch border for each child nodes
    child_nodes.at(0).pt_begin_ = pt_begin_;
    child_nodes.at(0).pt_end_ = pt_center;
    child_nodes.at(1).pt_begin_ = pt_top;
    child_nodes.at(1).pt_end_ = pt_right;
    child_nodes.at(2).pt_begin_ = pt_left;
    child_nodes.at(2).pt_end_ = pt_bottom;
    child_nodes.at(3).pt_begin_ = pt_center;
    child_nodes.at(3).pt_end_ = pt_end_;

    // Memory reservation for child nodes
    for(auto& node : child_nodes)
    {
        node.keypts_.reserve(keypts_.size());
    }

    // Distribute keypoints to child nodes
    for(const auto& keypt : keypts_)
    {
        unsigned int idx = 0;
        if(pt_begin_.x + half_x <= keypt.pt.x)
        {
            idx += 1;
        }
        if(pt_begin_.y + half_y <= keypt.pt.y)
        {
            idx += 2;
        }
        child_nodes.at(idx).keypts_.push_back(keypt);
    }

    return child_nodes;
}

struct orb_params
{
    orb_params() = delete;

    //! Constructor
    orb_params(const std::string& name, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr);

    //! Constructor
    // explicit orb_params(const YAML::Node& yaml_node);

    //! Destructor
    ~orb_params() = default;

    //! name (id for saving)
    const std::string name_;

    const float scale_factor_ = 1.2;
    const float log_scale_factor_ = std::log(1.2);
    const unsigned int num_levels_ = 8;
    const unsigned int ini_fast_thr_ = 20;
    const unsigned int min_fast_thr_ = 7;

    //! A list of the scale factor of each pyramid layer
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    //! A list of the sigma of each pyramid layer
    std::vector<float> level_sigma_sq_;
    std::vector<float> inv_level_sigma_sq_;

    //! Calculate scale factors
    static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of scale factors
    static std::vector<float> calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate squared sigmas at all levels
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of squared sigmas at all levels
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
};

orb_params::orb_params(const std::string& name, const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr)
    : name_(name), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)), num_levels_(num_levels),
      ini_fast_thr_(ini_fast_thr), min_fast_thr_(min_fast_thr)
{
    scale_factors_ = calc_scale_factors(num_levels_, scale_factor_);
    inv_scale_factors_ = calc_inv_scale_factors(num_levels_, scale_factor_);
    level_sigma_sq_ = calc_level_sigma_sq(num_levels_, scale_factor_);
    inv_level_sigma_sq_ = calc_inv_level_sigma_sq(num_levels_, scale_factor_);
}

std::vector<float> orb_params::calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor)
{
    std::vector<float> scale_factors(num_scale_levels, 1.0);
    for(unsigned int level = 1; level < num_scale_levels; ++level)
    {
        scale_factors.at(level) = scale_factor * scale_factors.at(level - 1);
    }

    return scale_factors;
}

std::vector<float> orb_params::calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor)
{
    std::vector<float> inv_scale_factors(num_scale_levels, 1.0);
    for(unsigned int level = 1; level < num_scale_levels; ++level)
    {
        inv_scale_factors.at(level) = (1.0f / scale_factor) * inv_scale_factors.at(level - 1);
    }
    return inv_scale_factors;
}

std::vector<float> orb_params::calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor)
{
    float scale_factor_at_level = 1.0;
    std::vector<float> level_sigma_sq(num_scale_levels, 1.0);
    for(unsigned int level = 1; level < num_scale_levels; ++level)
    {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
    }
    return level_sigma_sq;
}

std::vector<float> orb_params::calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor)
{
    float scale_factor_at_level = 1.0;
    std::vector<float> inv_level_sigma_sq(num_scale_levels, 1.0);
    for(unsigned int level = 1; level < num_scale_levels; ++level)
    {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }
    return inv_level_sigma_sq;
}

namespace util
{

static constexpr float _PI = 3.14159265358979f;
static constexpr float _PI_2 = _PI / 2.0f;
static constexpr float _TWO_PI = 2.0f * _PI;
static constexpr float _INV_TWO_PI = 1.0f / _TWO_PI;
static constexpr float _THREE_PI_2 = 3.0f * _PI_2;

inline float _cos(float v)
{
    constexpr float c1 = 0.99940307f;
    constexpr float c2 = -0.49558072f;
    constexpr float c3 = 0.03679168f;

    const float v2 = v * v;
    return c1 + v2 * (c2 + c3 * v2);
}

inline float cos(float v)
{
    v = v - cvFloor(v * _INV_TWO_PI) * _TWO_PI;
    v = (0.0f < v) ? v : -v;

    if(v < _PI_2)
    {
        return _cos(v);
    }
    else if(v < _PI)
    {
        return -_cos(_PI - v);
    }
    else if(v < _THREE_PI_2)
    {
        return -_cos(v - _PI);
    }
    else
    {
        return _cos(_TWO_PI - v);
    }
}

inline float sin(float v)
{
    return util::cos(_PI_2 - v);
}

} // namespace util

class orb_impl
{
  public:
    orb_impl();
    float ic_angle(const cv::Mat& image, const cv::Point2f& point) const;
    void compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const;

    //! BRIEF orientation
    static constexpr unsigned int fast_patch_size_ = 31;
    //! half size of FAST patch
    static constexpr int fast_half_patch_size_ = fast_patch_size_ / 2;

  private:
    //! Index limitation that used for calculating of keypoint orientation
    std::vector<int> u_max_;
};

orb_impl::orb_impl()
{
    // Preparate  for computation of orientation
    u_max_.resize(fast_half_patch_size_ + 1);
    const unsigned int vmax = std::floor(fast_half_patch_size_ * std::sqrt(2.0) / 2 + 1);
    const unsigned int vmin = std::ceil(fast_half_patch_size_ * std::sqrt(2.0) / 2);
    for(unsigned int v = 0; v <= vmax; ++v)
    {
        u_max_.at(v) = std::round(std::sqrt(fast_half_patch_size_ * fast_half_patch_size_ - v * v));
    }
    for(unsigned int v = fast_half_patch_size_, v0 = 0; vmin <= v; --v)
    {
        while(u_max_.at(v0) == u_max_.at(v0 + 1))
        {
            ++v0;
        }
        u_max_.at(v) = v0;
        ++v0;
    }
}

float orb_impl::ic_angle(const cv::Mat& image, const cv::Point2f& point) const
{
    int m_01 = 0, m_10 = 0;

    const uchar* const center = &image.at<uchar>(cvRound(point.y), cvRound(point.x));

    for(int u = -fast_half_patch_size_; u <= fast_half_patch_size_; ++u)
    {
        m_10 += u * center[u];
    }

    const auto step = static_cast<int>(image.step1());
    for(int v = 1; v <= fast_half_patch_size_; ++v)
    {
        int v_sum = 0;
        const int d = u_max_.at(v);
        for(int u = -d; u <= d; ++u)
        {
            const int val_plus = center[u + v * step];
            const int val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return cv::fastAtan2(m_01, m_10);
}

void orb_impl::compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const
{
    const float angle = keypt.angle * M_PI / 180.0;
    const float cos_angle = util::cos(angle);
    const float sin_angle = util::sin(angle);

    const uchar* const center = &image.at<uchar>(cvRound(keypt.pt.y), cvRound(keypt.pt.x));
    const auto step = static_cast<int>(image.step);

#ifdef USE_SSE_ORB
#if !((defined _MSC_VER && defined _M_X64) || (defined __GNUC__ && defined __x86_64__ && defined __SSE3__) || CV_SSE3)
#error "The processor is not compatible with SSE. Please configure the CMake with -DUSE_SSE_ORB=OFF."
#endif

    const __m128 _trig1 = _mm_set_ps(cos_angle, sin_angle, cos_angle, sin_angle);
    const __m128 _trig2 = _mm_set_ps(-sin_angle, cos_angle, -sin_angle, cos_angle);
    __m128 _point_pairs;
    __m128 _mul1;
    __m128 _mul2;
    __m128 _vs;
    __m128i _vi;
    alignas(16) int32_t ii[4];

#define COMPARE_ORB_POINTS(shift)                                                                                      \
    (_point_pairs = _mm_load_ps(orb_point_pairs + shift), _mul1 = _mm_mul_ps(_point_pairs, _trig1),                    \
     _mul2 = _mm_mul_ps(_point_pairs, _trig2), _vs = _mm_hadd_ps(_mul1, _mul2), _vi = _mm_cvtps_epi32(_vs),            \
     _mm_store_si128(reinterpret_cast<__m128i*>(ii), _vi),                                                             \
     center[ii[0] * step + ii[2]] < center[ii[1] * step + ii[3]])

#else

#define GET_VALUE(shift)                                                                                               \
    (center[cvRound(*(orb_point_pairs + shift) * sin_angle + *(orb_point_pairs + shift + 1) * cos_angle) * step +      \
            cvRound(*(orb_point_pairs + shift) * cos_angle - *(orb_point_pairs + shift + 1) * sin_angle)])

#define COMPARE_ORB_POINTS(shift) (GET_VALUE(shift) < GET_VALUE(shift + 2))

#endif

    // interval: (X, Y) x 2 points x 8 pairs = 32
    static constexpr unsigned interval = 32;

    for(unsigned int i = 0; i < orb_point_pairs_size / interval; ++i)
    {
        int32_t val = COMPARE_ORB_POINTS(i * interval);
        val |= COMPARE_ORB_POINTS(i * interval + 4) << 1;
        val |= COMPARE_ORB_POINTS(i * interval + 8) << 2;
        val |= COMPARE_ORB_POINTS(i * interval + 12) << 3;
        val |= COMPARE_ORB_POINTS(i * interval + 16) << 4;
        val |= COMPARE_ORB_POINTS(i * interval + 20) << 5;
        val |= COMPARE_ORB_POINTS(i * interval + 24) << 6;
        val |= COMPARE_ORB_POINTS(i * interval + 28) << 7;
        desc[i] = static_cast<uchar>(val);
    }

#undef GET_VALUE
#undef COMPARE_ORB_POINTS
}

class DistributedOrbOpenCvDetector::OrbExtractorPimpl
{
  public:
    OrbExtractorPimpl() = delete;

    OrbExtractorPimpl(const orb_params* orb_params, const unsigned int max_num_keypts,
                      const std::vector<std::vector<float>>& mask_rects = {});

    //! Extract keypoints and each descriptor of them
    void extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                 std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors);

    //! parameters for ORB extraction
    const orb_params* orb_params_;

    //! A vector of keypoint area represents mask area
    //! Each areas are denoted as form of [x_min / cols, x_max / cols, y_min / rows, y_max / rows]
    std::vector<std::vector<float>> mask_rects_;

    //! Image pyramid
    std::vector<cv::Mat> image_pyramid_;

  private:
    //! Create a mask matrix that constructed by rectangles
    void create_rectangle_mask(const unsigned int cols, const unsigned int rows);

    //! Compute image pyramid
    void compute_image_pyramid(const cv::Mat& image);

    //! Compute fast keypoints for cells in each image pyramid
    void compute_fast_keypoints(std::vector<std::vector<cv::KeyPoint>>& all_keypts, const cv::Mat& mask) const;

    //! Pick computed keypoints on the image uniformly
    std::vector<cv::KeyPoint> distribute_keypoints_via_tree(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                            int min_x, int max_x, int min_y, int max_y,
                                                            float inv_scale_factor) const;

    //! Initialize nodes that used for keypoint distribution tree
    std::list<orb_extractor_node> initialize_nodes(const std::vector<cv::KeyPoint>& keypts_to_distribute, int min_x,
                                                   int max_x, int min_y, int max_y) const;

    //! Assign child nodes to the all node list
    void assign_child_nodes(const std::array<orb_extractor_node, 4>& child_nodes, std::list<orb_extractor_node>& nodes,
                            std::vector<std::pair<int, orb_extractor_node*>>& leaf_nodes) const;

    //! Find keypoint which has maximum value of response
    std::vector<cv::KeyPoint> find_keypoints_with_max_response(std::list<orb_extractor_node>& nodes) const;

    //! Compute orientation for each keypoint
    void compute_orientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypts) const;

    //! Correct keypoint's position to comply with the scale
    void correct_keypoint_scale(std::vector<cv::KeyPoint>& keypts_at_level, const unsigned int level) const;

    //! Compute the gradient direction of pixel intensity in a circle around the point
    float ic_angle(const cv::Mat& image, const cv::Point2f& point) const;

    //! Compute orb descriptors for all keypoint
    void compute_orb_descriptors(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypts,
                                 cv::Mat& descriptors) const;

    //! Compute orb descriptor of a keypoint
    void compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const;

    //! Size of node occupied by one feature point
    unsigned int min_size_;

    //! size of maximum ORB patch radius
    static constexpr unsigned int orb_patch_radius_ = 19;

    //! rectangle mask has been already initialized or not
    bool mask_is_initialized_ = false;
    cv::Mat rect_mask_;

    orb_impl orb_impl_;
};

// ****************

DistributedOrbOpenCvDetector::OrbExtractorPimpl::OrbExtractorPimpl(const orb_params* orb_params,
                                                                   const unsigned int min_size,
                                                                   const std::vector<std::vector<float>>& mask_rects)
    : orb_params_(orb_params), mask_rects_(mask_rects), min_size_(min_size)
{
    // resize buffers according to the number of levels
    image_pyramid_.resize(orb_params_->num_levels_);
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::extract(const cv::_InputArray& in_image,
                                                              const cv::_InputArray& in_image_mask,
                                                              std::vector<cv::KeyPoint>& keypts,
                                                              const cv::_OutputArray& out_descriptors)
{
    if(in_image.empty())
    {
        return;
    }

    // get cv::Mat of image
    const auto image = in_image.getMat();
    assert(image.type() == CV_8UC1);

    // build image pyramid
    compute_image_pyramid(image);

    // mask initialization
    if(!mask_is_initialized_ && !mask_rects_.empty())
    {
        create_rectangle_mask(image.cols, image.rows);
        mask_is_initialized_ = true;
    }

    std::vector<std::vector<cv::KeyPoint>> all_keypts;

    // select mask to use
    if(!in_image_mask.empty())
    {
        // Use image_mask if it is available
        const auto image_mask = in_image_mask.getMat();
        assert(image_mask.type() == CV_8UC1);
        compute_fast_keypoints(all_keypts, image_mask);
    }
    else if(!rect_mask_.empty())
    {
        // Use rectangle mask if it is available and image_mask is not used
        assert(rect_mask_.type() == CV_8UC1);
        compute_fast_keypoints(all_keypts, rect_mask_);
    }
    else
    {
        // Do not use any mask if all masks are unavailable
        compute_fast_keypoints(all_keypts, cv::Mat());
    }

    cv::Mat descriptors;

    unsigned int num_keypts = 0;
    for(unsigned int level = 0; level < orb_params_->num_levels_; ++level)
    {
        num_keypts += all_keypts.at(level).size();
    }

    if(num_keypts == 0)
    {
        out_descriptors.release();
    }
    else
    {
        out_descriptors.create(num_keypts, 32, CV_8U);
        descriptors = out_descriptors.getMat();
    }

    keypts.clear();
    keypts.reserve(num_keypts);

    unsigned int offset = 0;
    for(unsigned int level = 0; level < orb_params_->num_levels_; ++level)
    {
        auto& keypts_at_level = all_keypts.at(level);
        const auto num_keypts_at_level = keypts_at_level.size();

        if(num_keypts_at_level == 0)
        {
            continue;
        }

        cv::Mat blurred_image = image_pyramid_.at(level).clone();
        cv::GaussianBlur(blurred_image, blurred_image, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

        cv::Mat descriptors_at_level = descriptors.rowRange(offset, offset + num_keypts_at_level);
        compute_orb_descriptors(blurred_image, keypts_at_level, descriptors_at_level);

        offset += num_keypts_at_level;

        correct_keypoint_scale(keypts_at_level, level);

        keypts.insert(keypts.end(), keypts_at_level.begin(), keypts_at_level.end());
    }
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::create_rectangle_mask(const unsigned int cols,
                                                                            const unsigned int rows)
{
    if(rect_mask_.empty())
    {
        rect_mask_ = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
    }
    // draw masks
    for(const auto& mask_rect : mask_rects_)
    {
        // draw black rectangle
        const unsigned int x_min = std::round(cols * mask_rect.at(0));
        const unsigned int x_max = std::round(cols * mask_rect.at(1));
        const unsigned int y_min = std::round(rows * mask_rect.at(2));
        const unsigned int y_max = std::round(rows * mask_rect.at(3));
        cv::rectangle(rect_mask_, cv::Point2i(x_min, y_min), cv::Point2i(x_max, y_max), cv::Scalar(0), -1, cv::LINE_AA);
    }
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::compute_image_pyramid(const cv::Mat& image)
{
    image_pyramid_.at(0) = image;
    for(unsigned int level = 1; level < orb_params_->num_levels_; ++level)
    {
        // determine the size of an image
        const double scale = orb_params_->scale_factors_.at(level);
        const cv::Size size(std::round(image.cols * 1.0 / scale), std::round(image.rows * 1.0 / scale));
        // resize
        cv::resize(image_pyramid_.at(level - 1), image_pyramid_.at(level), size, 0, 0, cv::INTER_LINEAR);
    }
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::compute_fast_keypoints(
    std::vector<std::vector<cv::KeyPoint>>& all_keypts, const cv::Mat& mask) const
{
    all_keypts.resize(orb_params_->num_levels_);

    // An anonymous function which checks mask(image or rectangle)
    auto is_in_mask = [&mask](const unsigned int y, const unsigned int x, const float scale_factor)
    { return mask.at<unsigned char>(y * scale_factor, x * scale_factor) == 0; };

    constexpr unsigned int overlap = 6;
    constexpr unsigned int cell_size = 64;

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for(int64_t level = 0; level < orb_params_->num_levels_; ++level)
    {
        const float scale_factor = orb_params_->scale_factors_.at(level);

        constexpr unsigned int min_border_x = orb_patch_radius_;
        constexpr unsigned int min_border_y = orb_patch_radius_;
        const unsigned int max_border_x = image_pyramid_.at(level).cols - orb_patch_radius_;
        const unsigned int max_border_y = image_pyramid_.at(level).rows - orb_patch_radius_;

        const unsigned int width = max_border_x - min_border_x;
        const unsigned int height = max_border_y - min_border_y;

        const unsigned int num_cols = std::ceil(width / cell_size) + 1;
        const unsigned int num_rows = std::ceil(height / cell_size) + 1;

        std::vector<cv::KeyPoint> keypts_to_distribute;

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for(int64_t i = 0; i < num_rows; ++i)
        {
            const unsigned int min_y = min_border_y + i * cell_size;
            if(max_border_y - overlap <= min_y)
            {
                continue;
            }
            unsigned int max_y = min_y + cell_size + overlap;
            if(max_border_y < max_y)
            {
                max_y = max_border_y;
            }

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
            for(int64_t j = 0; j < num_cols; ++j)
            {
                const unsigned int min_x = min_border_x + j * cell_size;
                if(max_border_x - overlap <= min_x)
                {
                    continue;
                }
                unsigned int max_x = min_x + cell_size + overlap;
                if(max_border_x < max_x)
                {
                    max_x = max_border_x;
                }

                // Pass FAST computation if one of the corners of a patch is in the mask
                if(!mask.empty())
                {
                    if(is_in_mask(min_y, min_x, scale_factor) || is_in_mask(max_y, min_x, scale_factor) ||
                       is_in_mask(min_y, max_x, scale_factor) || is_in_mask(max_y, max_x, scale_factor))
                    {
                        continue;
                    }
                }

                std::vector<cv::KeyPoint> keypts_in_cell;
                cv::FAST(image_pyramid_.at(level).rowRange(min_y, max_y).colRange(min_x, max_x), keypts_in_cell,
                         orb_params_->ini_fast_thr_, true);

                // Re-compute FAST keypoint with reduced threshold if enough keypoint was not got
                if(keypts_in_cell.empty())
                {
                    cv::FAST(image_pyramid_.at(level).rowRange(min_y, max_y).colRange(min_x, max_x), keypts_in_cell,
                             orb_params_->min_fast_thr_, true);
                }

                if(keypts_in_cell.empty())
                {
                    continue;
                }

                // Collect keypoints for every scale
#ifdef USE_OPENMP
#pragma omp critical
#endif
                {
                    for(auto& keypt : keypts_in_cell)
                    {
                        keypt.pt.x += j * cell_size;
                        keypt.pt.y += i * cell_size;
                        // Check if the keypoint is in the mask
                        if(!mask.empty() &&
                           is_in_mask(min_border_y + keypt.pt.y, min_border_x + keypt.pt.x, scale_factor))
                        {
                            continue;
                        }
                        keypts_to_distribute.push_back(keypt);
                    }
                }
            }
        }

        std::vector<cv::KeyPoint>& keypts_at_level = all_keypts.at(level);

        // Distribute keypoints via tree
        keypts_at_level = distribute_keypoints_via_tree(keypts_to_distribute, min_border_x, max_border_x, min_border_y,
                                                        max_border_y, scale_factor);

        // Keypoint size is patch size modified by the scale factor
        const unsigned int scaled_patch_size = orb_impl_.fast_patch_size_ * scale_factor;

        for(auto& keypt : keypts_at_level)
        {
            // Translation correction (scale will be corrected after ORB description)
            keypt.pt.x += min_border_x;
            keypt.pt.y += min_border_y;
            // Set the other information
            keypt.octave = level;
            keypt.size = scaled_patch_size;
        }
    }

    // Compute orientations
    for(unsigned int level = 0; level < orb_params_->num_levels_; ++level)
    {
        compute_orientation(image_pyramid_.at(level), all_keypts.at(level));
    }
}

std::vector<cv::KeyPoint> DistributedOrbOpenCvDetector::OrbExtractorPimpl::distribute_keypoints_via_tree(
    const std::vector<cv::KeyPoint>& keypts_to_distribute, const int min_x, const int max_x, const int min_y,
    const int max_y, const float scale_factor) const
{
    auto nodes = initialize_nodes(keypts_to_distribute, min_x, max_x, min_y, max_y);

    // Forkable leaf nodes list
    // The pool is used when a forking makes nodes more than a limited number
    std::vector<std::pair<int, orb_extractor_node*>> leaf_nodes_pool;
    leaf_nodes_pool.reserve(nodes.size() * 10);

    while(true)
    {
        const unsigned int prev_size = nodes.size();

        auto iter = nodes.begin();
        leaf_nodes_pool.clear();

        // Fork node and remove the old one from nodes
        while(iter != nodes.end())
        {
            if(iter->keypts_.size() == 1 || iter->size() * scale_factor * scale_factor <= min_size_)
            {
                iter++;
                continue;
            }

            // Divide node and assign to the leaf node pool
            const auto child_nodes = iter->divide_node();
            assign_child_nodes(child_nodes, nodes, leaf_nodes_pool);
            // Remove the old node
            iter = nodes.erase(iter);
        }

        // Stop iteration when the number of nodes is over the designated size or new node is not generated
        if(nodes.size() == prev_size)
        {
            break;
        }
    }

    return find_keypoints_with_max_response(nodes);
}

std::list<orb_extractor_node>
DistributedOrbOpenCvDetector::OrbExtractorPimpl::initialize_nodes(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                                  const int min_x, const int max_x, const int min_y,
                                                                  const int max_y) const
{
    // The aspect ratio of the target area for keypoint detection
    const auto ratio = static_cast<double>(max_x - min_x) / (max_y - min_y);
    // The width and height of the patches allocated to the initial node
    double delta_x, delta_y;
    // The number of columns or rows
    unsigned int num_x_grid, num_y_grid;

    if(ratio > 1)
    {
        // If the aspect ratio is greater than 1, the patches are made in a horizontal direction
        num_x_grid = std::round(ratio);
        num_y_grid = 1;
        delta_x = static_cast<double>(max_x - min_x) / num_x_grid;
        delta_y = max_y - min_y;
    }
    else
    {
        // If the aspect ratio is equal to or less than 1, the patches are made in a vertical direction
        num_x_grid = 1;
        num_y_grid = std::round(1 / ratio);
        delta_x = max_x - min_y;
        delta_y = static_cast<double>(max_y - min_y) / num_y_grid;
    }

    // The number of the initial nodes
    const unsigned int num_initial_nodes = num_x_grid * num_y_grid;

    // A list of node
    std::list<orb_extractor_node> nodes;

    // Initial node objects
    std::vector<orb_extractor_node*> initial_nodes;
    initial_nodes.resize(num_initial_nodes);

    // Create initial node substances
    for(unsigned int i = 0; i < num_initial_nodes; ++i)
    {
        orb_extractor_node node;

        // x / y index of the node's patch in the grid
        const unsigned int ix = i % num_x_grid;
        const unsigned int iy = i / num_x_grid;

        node.pt_begin_ = cv::Point2i(delta_x * ix, delta_y * iy);
        node.pt_end_ = cv::Point2i(delta_x * (ix + 1), delta_y * (iy + 1));
        node.keypts_.reserve(keypts_to_distribute.size());

        nodes.push_back(node);
        initial_nodes.at(i) = &nodes.back();
    }

    // Assign all keypoints to initial nodes which own keypoint's position
    for(const auto& keypt : keypts_to_distribute)
    {
        // x / y index of the patch where the keypt is placed
        const unsigned int ix = keypt.pt.x / delta_x;
        const unsigned int iy = keypt.pt.y / delta_y;

        const unsigned int node_idx = ix + iy * num_x_grid;
        initial_nodes.at(node_idx)->keypts_.push_back(keypt);
    }

    auto iter = nodes.begin();
    while(iter != nodes.end())
    {
        // Remove empty nodes
        if(iter->keypts_.empty())
        {
            iter = nodes.erase(iter);
            continue;
        }
        iter++;
    }

    return nodes;
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::assign_child_nodes(
    const std::array<orb_extractor_node, 4>& child_nodes, std::list<orb_extractor_node>& nodes,
    std::vector<std::pair<int, orb_extractor_node*>>& leaf_nodes) const
{
    for(const auto& child_node : child_nodes)
    {
        if(child_node.keypts_.empty())
        {
            continue;
        }
        nodes.push_front(child_node);
        if(child_node.keypts_.size() == 1)
        {
            continue;
        }
        leaf_nodes.emplace_back(std::make_pair(child_node.keypts_.size(), &nodes.front()));
        // Keep the self iterator to remove from std::list randomly
        nodes.front().iter_ = nodes.begin();
    }
}

std::vector<cv::KeyPoint> DistributedOrbOpenCvDetector::OrbExtractorPimpl::find_keypoints_with_max_response(
    std::list<orb_extractor_node>& nodes) const
{
    // A vector contains result keypoint
    std::vector<cv::KeyPoint> result_keypts;
    result_keypts.reserve(nodes.size());

    // Store keypoints which has maximum response in the node patch
    for(auto& node : nodes)
    {
        auto& node_keypts = node.keypts_;
        auto& keypt = node_keypts.at(0);
        double max_response = keypt.response;

        for(unsigned int k = 1; k < node_keypts.size(); ++k)
        {
            if(node_keypts.at(k).response > max_response)
            {
                keypt = node_keypts.at(k);
                max_response = node_keypts.at(k).response;
            }
        }

        result_keypts.push_back(keypt);
    }

    return result_keypts;
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::compute_orientation(const cv::Mat& image,
                                                                          std::vector<cv::KeyPoint>& keypts) const
{
    for(auto& keypt : keypts)
    {
        keypt.angle = ic_angle(image, keypt.pt);
    }
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::correct_keypoint_scale(std::vector<cv::KeyPoint>& keypts_at_level,
                                                                             const unsigned int level) const
{
    if(level == 0)
    {
        return;
    }

    const float scale_at_level = orb_params_->scale_factors_.at(level);
    for(auto& keypt_at_level : keypts_at_level)
    {
        keypt_at_level.pt *= scale_at_level;
    }
}

DistributedOrbOpenCvDetector::DistributedOrbOpenCvDetector()
{
    // TODO: its a bug
    orb_params* params = new orb_params("orb", 1.2f, 8, 20, 7);

    pimpl = std::make_unique<OrbExtractorPimpl>(params, 1000);
}

DistributedOrbOpenCvDetector::~DistributedOrbOpenCvDetector() = default;
std::vector<OrbKeypoint> DistributedOrbOpenCvDetector::detect(const RgbFrame& sensorData)
{

    auto gray = toGrayScale(sensorData);
    cv::Mat cvGray{gray.size.height, gray.size.width, CV_8UC1, const_cast<std::uint8_t*>(gray.data.data())};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    pimpl->extract(cvGray, cv::Mat(), keypoints, descriptors);

    std::vector<OrbKeypoint> result;
    Id id = 0;

    for(const auto& cvKeypoint : keypoints)
    {
        OrbKeypoint keypoint;
        keypoint.keypoint.id = id;
        keypoint.keypoint.coordinates.x() = cvKeypoint.pt.x;
        keypoint.keypoint.coordinates.y() = cvKeypoint.pt.y;

        const auto* const ptr = descriptors.ptr<std::uint8_t>(id++);
        std::copy(ptr, ptr + 32, std::begin(keypoint.descriptor));

        result.push_back(keypoint);
    }

    spdlog::info("Size of desc: {} x {}", descriptors.rows, descriptors.cols);
    cv::drawKeypoints(cvGray, keypoints, cvGray);
    cv::imshow("kpts", cvGray);
    cv::waitKey(1);

    return result;
}

float DistributedOrbOpenCvDetector::OrbExtractorPimpl::ic_angle(const cv::Mat& image, const cv::Point2f& point) const
{
    return orb_impl_.ic_angle(image, point);
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::compute_orb_descriptors(const cv::Mat& image,
                                                                              const std::vector<cv::KeyPoint>& keypts,
                                                                              cv::Mat& descriptors) const
{
    descriptors = cv::Mat::zeros(keypts.size(), 32, CV_8UC1);

    for(unsigned int i = 0; i < keypts.size(); ++i)
    {
        compute_orb_descriptor(keypts.at(i), image, descriptors.ptr(i));
    }
}

void DistributedOrbOpenCvDetector::OrbExtractorPimpl::compute_orb_descriptor(const cv::KeyPoint& keypt,
                                                                             const cv::Mat& image, uchar* desc) const
{
    orb_impl_.compute_orb_descriptor(keypt, image, desc);
}

} // namespace mslam
