"""조사 결과 문서화 모듈."""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_DOCS_DIR = _PROJECT_ROOT / "docs"


def write_report(
    title: str,
    sections: list[dict[str, str]],
    author: str = "Research Agent",
) -> str:
    """마크다운 보고서 생성.

    Args:
        title: 보고서 제목
        sections: [{"heading": "...", "content": "..."}] 형태
        author: 작성자

    Returns:
        생성된 마크다운 문자열
    """
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = [
        f"# {title}",
        "",
        f"> 작성: {author} | {now}",
        "",
        "---",
        "",
    ]

    for section in sections:
        heading = section.get("heading", "")
        content = section.get("content", "")
        if heading:
            lines.append(f"## {heading}")
            lines.append("")
        if content:
            lines.append(content)
            lines.append("")

    return "\n".join(lines)


def format_search_results(results: list[dict]) -> str:
    """검색 결과를 마크다운 형식으로 포맷팅."""
    if not results:
        return "_검색 결과 없음_"

    lines = []
    for i, r in enumerate(results, 1):
        title = r.get("title", "제목 없음")
        url = r.get("url", "")
        snippet = r.get("snippet", "")

        lines.append(f"### {i}. {title}")
        if url:
            lines.append(f"- URL: {url}")
        if snippet:
            lines.append(f"- {snippet}")
        lines.append("")

    return "\n".join(lines)


def format_paper_results(papers: list[dict]) -> str:
    """논문 검색 결과를 마크다운 형식으로 포맷팅."""
    if not papers:
        return "_논문 검색 결과 없음_"

    lines = []
    for i, p in enumerate(papers, 1):
        title = p.get("title", "제목 없음")
        authors = ", ".join(p.get("authors", [])[:3])
        if len(p.get("authors", [])) > 3:
            authors += " et al."
        summary = p.get("summary", "")
        url = p.get("url", "")
        published = p.get("published", "")

        lines.append(f"### {i}. {title}")
        if authors:
            lines.append(f"- 저자: {authors}")
        if published:
            lines.append(f"- 발표: {published}")
        if url:
            lines.append(f"- URL: {url}")
        if summary:
            lines.append(f"- 요약: {summary[:200]}...")
        lines.append("")

    return "\n".join(lines)


def save_to_project(filename: str, content: str) -> str:
    """프로젝트 docs/ 폴더에 파일 저장.

    Args:
        filename: 파일명 (확장자 포함)
        content: 파일 내용

    Returns:
        저장된 파일의 절대 경로
    """
    _DOCS_DIR.mkdir(parents=True, exist_ok=True)

    # 파일명 안전 처리
    safe_name = "".join(c if c.isalnum() or c in ".-_ " else "_" for c in filename)
    if not safe_name.endswith(".md"):
        safe_name += ".md"

    filepath = _DOCS_DIR / safe_name
    filepath.write_text(content, encoding="utf-8")
    logger.info("문서 저장: %s", filepath)
    return str(filepath)


def list_documents() -> list[dict[str, str]]:
    """docs/ 폴더의 문서 목록."""
    if not _DOCS_DIR.exists():
        return []

    docs = []
    for f in sorted(_DOCS_DIR.glob("*.md")):
        stat = f.stat()
        docs.append({
            "name": f.name,
            "path": str(f),
            "size": f"{stat.st_size} bytes",
            "modified": datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M"),
        })
    return docs
