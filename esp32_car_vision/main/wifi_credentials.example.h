#pragma once
/*
 * 模板：把本文件复制为 wifi_credentials.h 再填（wifi_credentials.h 已被 .gitignore 排除）
 *
 * 情况A：校园网 802.1X 企业认证（Tsinghua-Secure 等）→ WIFI_USE_ENTERPRISE = 1
 *        账号 = 学号，密码 = 校园网密码（登录 net.tsinghua.edu.cn 那个）
 * 情况B：普通家庭 WiFi / 手机热点 → WIFI_USE_ENTERPRISE = 0，填 WIFI_PASSWORD
 */

#define WIFI_USE_ENTERPRISE   1

#define WIFI_SSID             "Tsinghua-Secure"

/* ---- 情况A：校园网 802.1X（WIFI_USE_ENTERPRISE=1 时生效） ---- */
#define EAP_IDENTITY          "CHANGE_ME_STUDENT_ID"
#define EAP_USERNAME          "CHANGE_ME_STUDENT_ID"
#define EAP_PASSWORD          "CHANGE_ME_CAMPUS_PASSWORD"

/* ---- 情况B：普通 WiFi 密码（WIFI_USE_ENTERPRISE=0 时生效） ---- */
#define WIFI_PASSWORD         "CHANGE_ME_WIFI_PASSWORD"
