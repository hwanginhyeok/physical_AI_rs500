"""웹 검색 모듈 (논문/기술자료 수집)."""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from urllib.parse import quote_plus

logger = logging.getLogger(__name__)

try:
    import aiohttp
except ImportError:
    aiohttp = None  # type: ignore[assignment]
    logger.warning("aiohttp가 설치되지 않았습니다. pip install aiohttp")


@dataclass
class SearchResult:
    """검색 결과 항목."""

    title: str
    url: str
    snippet: str = ""
    source: str = ""

    def to_dict(self) -> dict:
        return {"title": self.title, "url": self.url, "snippet": self.snippet, "source": self.source}


@dataclass
class PaperResult:
    """학술 논문 검색 결과."""

    title: str
    authors: list[str] = field(default_factory=list)
    summary: str = ""
    url: str = ""
    published: str = ""

    def to_dict(self) -> dict:
        return {
            "title": self.title,
            "authors": self.authors,
            "summary": self.summary[:300],
            "url": self.url,
            "published": self.published,
        }


def _require_aiohttp() -> None:
    if aiohttp is None:
        raise RuntimeError("aiohttp가 필요합니다: pip install aiohttp")


async def search(query: str, num_results: int = 5) -> list[SearchResult]:
    """DuckDuckGo Lite를 이용한 웹 검색.

    Args:
        query: 검색어
        num_results: 결과 수 (최대 10)

    Returns:
        SearchResult 리스트
    """
    _require_aiohttp()
    url = "https://lite.duckduckgo.com/lite/"
    params = {"q": query, "kl": "kr-kr"}

    results: list[SearchResult] = []
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(url, data=params, timeout=aiohttp.ClientTimeout(total=15)) as resp:
                if resp.status != 200:
                    logger.error("DuckDuckGo 검색 실패: HTTP %s", resp.status)
                    return results
                html = await resp.text()

        # DuckDuckGo Lite HTML에서 결과 추출 (간단한 파싱)
        import re

        links = re.findall(
            r'class="result-link"[^>]*href="([^"]+)"[^>]*>([^<]+)</a>',
            html,
        )
        snippets = re.findall(r'class="result-snippet">([^<]+)<', html)

        for i, (link_url, title) in enumerate(links[:num_results]):
            snippet = snippets[i] if i < len(snippets) else ""
            results.append(
                SearchResult(
                    title=title.strip(),
                    url=link_url.strip(),
                    snippet=snippet.strip(),
                    source="duckduckgo",
                )
            )
    except Exception as e:
        logger.error("웹 검색 오류: %s", e)

    logger.info("웹 검색 '%s': %d건 결과", query, len(results))
    return results


async def search_papers(query: str, max_results: int = 5) -> list[PaperResult]:
    """arXiv API를 이용한 학술 논문 검색.

    Args:
        query: 검색어
        max_results: 결과 수

    Returns:
        PaperResult 리스트
    """
    _require_aiohttp()
    encoded = quote_plus(query)
    url = f"http://export.arxiv.org/api/query?search_query=all:{encoded}&start=0&max_results={max_results}"

    papers: list[PaperResult] = []
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url, timeout=aiohttp.ClientTimeout(total=15)) as resp:
                if resp.status != 200:
                    logger.error("arXiv 검색 실패: HTTP %s", resp.status)
                    return papers
                xml_text = await resp.text()

        root = ET.fromstring(xml_text)
        ns = {"atom": "http://www.w3.org/2005/Atom"}

        for entry in root.findall("atom:entry", ns):
            title = entry.findtext("atom:title", "", ns).strip().replace("\n", " ")
            summary = entry.findtext("atom:summary", "", ns).strip().replace("\n", " ")
            published = entry.findtext("atom:published", "", ns)[:10]

            authors = [
                a.findtext("atom:name", "", ns)
                for a in entry.findall("atom:author", ns)
            ]

            link_elem = entry.find('atom:link[@type="text/html"]', ns)
            paper_url = link_elem.get("href", "") if link_elem is not None else ""
            if not paper_url:
                id_elem = entry.findtext("atom:id", "", ns)
                paper_url = id_elem

            papers.append(
                PaperResult(
                    title=title,
                    authors=authors,
                    summary=summary,
                    url=paper_url,
                    published=published,
                )
            )
    except Exception as e:
        logger.error("논문 검색 오류: %s", e)

    logger.info("논문 검색 '%s': %d건 결과", query, len(papers))
    return papers


async def fetch_page(url: str) -> str:
    """웹 페이지 텍스트 내용 수집.

    Returns:
        페이지 텍스트 (HTML 태그 제거, 최대 5000자)
    """
    _require_aiohttp()
    try:
        async with aiohttp.ClientSession() as session:
            headers = {"User-Agent": "Mozilla/5.0 (research-agent)"}
            async with session.get(url, headers=headers, timeout=aiohttp.ClientTimeout(total=15)) as resp:
                if resp.status != 200:
                    return f"[HTTP {resp.status}]"
                html = await resp.text()

        import re
        # 스크립트/스타일 제거
        text = re.sub(r"<(script|style)[^>]*>.*?</\1>", "", html, flags=re.DOTALL)
        # HTML 태그 제거
        text = re.sub(r"<[^>]+>", " ", text)
        # 공백 정리
        text = re.sub(r"\s+", " ", text).strip()
        return text[:5000]
    except Exception as e:
        return f"[페이지 수집 오류: {e}]"
