//! OBS 上传：POST /api/obs/temporary/url 换签名 URL → PUT JSON。

use super::client::{get_field, parse_data_field, ApiClient};
use serde_json::{json, Value};

pub const OBS_SIGN_PATH: &str = "/api/obs/temporary/url";

/// 换取签名 URL。
pub fn sign_url(client: &mut ApiClient, method: &str, key: &str) -> Result<String, String> {
    let body = json!({
        "bucketName": "iydsj-hbase-hot",
        "objectKey": key,
        "method": method,
        "contentType": "application/json",
    })
    .to_string();
    let biz = client.call("POST", OBS_SIGN_PATH, &body, &[])?;
    // business.data 可能是 JSON 字符串 {"signedUrl": ...}
    let data = parse_data_field(&biz);
    let signed = data
        .get("signedUrl")
        .and_then(|v| v.as_str())
        .or_else(|| get_field(&biz, "signedUrl").and_then(|v| v.as_str()))
        .ok_or("OBS 签名响应缺 signedUrl")?;
    Ok(signed.to_string())
}

/// 获取可用于兜底上传的候选代理列表（优先环境变量与本机 127.0.0.1 代理，随后是备用通道）。
fn get_proxy_candidates() -> Vec<String> {
    let mut list = Vec::new();
    for key in ["HTTP_PROXY", "http_proxy", "ALL_PROXY", "all_proxy"] {
        if let Ok(v) = std::env::var(key) {
            let trimmed = v.trim();
            if !trimmed.is_empty() {
                let formatted = if !trimmed.starts_with("http://") && !trimmed.starts_with("socks5://") {
                    format!("http://{trimmed}")
                } else {
                    trimmed.to_string()
                };
                if !list.contains(&formatted) {
                    list.push(formatted);
                }
            }
        }
    }
    let local = "http://127.0.0.1:7890".to_string();
    if !list.contains(&local) {
        list.push(local);
    }
    let remote = "http://100.120.44.46:7890".to_string();
    if !list.contains(&remote) {
        list.push(remote);
    }
    list
}

/// PUT 上传 OBS 对象（带自动重试与代理故障回退，彻底解决校园网/部分网络对腾讯云 COS 重置 10054 导致数据缺失）。
pub fn put_object(signed_url: &str, payload: &[u8], log: &mut dyn FnMut(&str)) -> Result<(), String> {
    let mut last_err = String::new();

    // 1. 优先尝试常规直连
    let direct_agent = super::client::make_agent();
    for attempt in 1..=2 {
        match direct_agent
            .put(signed_url)
            .set("Content-Type", "application/json")
            .send_bytes(payload)
        {
            Ok(resp) => {
                log(&format!("[obs] PUT {} -> {}", shorten(signed_url), resp.status()));
                return Ok(());
            }
            Err(e) => {
                last_err = super::client::ureq_err(e);
                log(&format!("[obs] PUT 第 {attempt}/2 次直连尝试失败: {last_err}"));
                std::thread::sleep(std::time::Duration::from_millis(600));
            }
        }
    }

    // 2. 若直连遭遇 10054 (Connection was reset) 或网络拦截，尝试探测可用代理通道进行兜底上传
    let proxy_candidates = get_proxy_candidates();
    for proxy_url in &proxy_candidates {
        if let Ok(proxy) = ureq::Proxy::new(proxy_url) {
            let proxy_agent = ureq::AgentBuilder::new()
                .timeout(std::time::Duration::from_secs(12))
                .proxy(proxy)
                .build();
            log(&format!("[obs] 正在尝试通过代理通道 {proxy_url} 兜底上传…"));
            match proxy_agent
                .put(signed_url)
                .set("Content-Type", "application/json")
                .send_bytes(payload)
            {
                Ok(resp) => {
                    log(&format!("[obs] √ 通过代理通道上传成功: {} -> {}", shorten(signed_url), resp.status()));
                    return Ok(());
                }
                Err(e) => {
                    let err_str = super::client::ureq_err(e);
                    log(&format!("[obs] 代理通道 {proxy_url} 上传失败: {err_str}"));
                }
            }
        }
    }

    Err(last_err)
}

/// 上传到两个 objectKey（详情页 + 兜底路径）。
pub fn upload_both_keys(
    client: &mut ApiClient,
    keys: &[String],
    payload: &[u8],
    log: &mut dyn FnMut(&str),
) -> usize {
    let mut ok = 0;
    for key in keys {
        match sign_url(client, "Put", key).and_then(|url| put_object(&url, payload, log)) {
            Ok(()) => ok += 1,
            Err(e) => log(&format!("[obs] PUT {key} 失败: {e}")),
        }
    }
    ok
}

fn shorten(url: &str) -> &str {
    // 只显示 objectKey 部分，避免日志过长
    let start = url.find("run_data").unwrap_or(0);
    let end = (start + 60).min(url.len());
    &url[start..end]
}

/// 回读验证（GET 签名，带 10054 自动重试与代理回退）。
#[allow(dead_code)]
pub fn fetch_object(client: &mut ApiClient, key: &str, log: &mut dyn FnMut(&str)) -> Result<Value, String> {
    let signed = sign_url(client, "Get", key)?;
    let agent = super::client::make_agent();
    let text = match agent.get(&signed).call() {
        Ok(resp) => resp.into_string().unwrap_or_default(),
        Err(e) => {
            let err_str = super::client::ureq_err(e);
            if err_str.contains("10054") || err_str.contains("reset") || err_str.contains("Connection") {
                log("⚠ [obs] GET 回读检测到 10054 重置，尝试通过代理通道回读…");
                let proxy_candidates = get_proxy_candidates();
                let mut proxy_text = None;
                for proxy_url in &proxy_candidates {
                    if let Ok(proxy) = ureq::Proxy::new(proxy_url) {
                        let proxy_agent = ureq::AgentBuilder::new()
                            .timeout(std::time::Duration::from_secs(10))
                            .proxy(proxy)
                            .build();
                        log(&format!("[obs] 尝试通过代理 {proxy_url} 回读…"));
                        match proxy_agent.get(&signed).call() {
                            Ok(resp) => {
                                if let Ok(txt) = resp.into_string() {
                                    log(&format!("√ [obs] 通过代理 {proxy_url} 回读成功，字节数={}", txt.len()));
                                    proxy_text = Some(txt);
                                    break;
                                }
                            }
                            Err(e) => {
                                log(&format!("⚠ [obs] 代理 {proxy_url} 回读失败: {e}"));
                            }
                        }
                    }
                }
                proxy_text.ok_or(err_str)?
            } else {
                return Err(err_str);
            }
        }
    };
    log(&format!("[obs] GET 回读 {} 字节", text.len()));
    serde_json::from_str(&text).map_err(|e| format!("OBS 对象解析失败: {e}"))
}

