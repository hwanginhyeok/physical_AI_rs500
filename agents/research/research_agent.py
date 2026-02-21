"""선행조사 통합 에이전트."""

from __future__ import annotations

import logging

from ..core.base_agent import BaseAgent
from ..core.message_bus import MessageBus
from ..core.task import Task
from . import document_writer, web_searcher

logger = logging.getLogger(__name__)


class ResearchAgent(BaseAgent):
    """웹 검색 + 논문 조사 + 문서화를 수행하는 에이전트."""

    COMMANDS = {
        "search",
        "search_papers",
        "fetch_page",
        "write_report",
        "save_document",
        "list_documents",
        "research",  # 통합 워크플로우
    }

    def __init__(self, bus: MessageBus) -> None:
        super().__init__("research", bus)

    async def handle_task(self, task: Task) -> object:
        params = task.result if isinstance(task.result, dict) else {}
        command = params.pop("command", None) or task.title

        if command == "search":
            query = params.get("query", task.description)
            num = params.get("num_results", 5)
            results = await web_searcher.search(query, num)
            return [r.to_dict() for r in results]

        if command == "search_papers":
            query = params.get("query", task.description)
            max_results = params.get("max_results", 5)
            papers = await web_searcher.search_papers(query, max_results)
            return [p.to_dict() for p in papers]

        if command == "fetch_page":
            url = params.get("url", task.description)
            return await web_searcher.fetch_page(url)

        if command == "write_report":
            title = params.get("title", "조사 보고서")
            sections = params.get("sections", [])
            return document_writer.write_report(title, sections)

        if command == "save_document":
            filename = params.get("filename", "report.md")
            content = params.get("content", "")
            return document_writer.save_to_project(filename, content)

        if command == "list_documents":
            return document_writer.list_documents()

        if command == "research":
            return await self._research_workflow(
                query=params.get("query", task.description),
                save=params.get("save", True),
            )

        raise ValueError(
            f"알 수 없는 연구 명령: {command}\n"
            f"사용 가능한 명령: {', '.join(sorted(self.COMMANDS))}"
        )

    async def _research_workflow(self, query: str, save: bool = True) -> dict:
        """통합 조사 워크플로우: 웹 검색 + 논문 검색 + 보고서 생성.

        Args:
            query: 조사 주제
            save: docs/에 보고서 저장 여부

        Returns:
            {"web": [...], "papers": [...], "report_path": "..."}
        """
        logger.info("[research] 통합 조사 시작: %s", query)

        # 1. 웹 검색
        web_results = await web_searcher.search(query, num_results=5)
        web_dicts = [r.to_dict() for r in web_results]

        # 2. 논문 검색
        paper_results = await web_searcher.search_papers(query, max_results=5)
        paper_dicts = [p.to_dict() for p in paper_results]

        # 3. 보고서 생성
        sections = [
            {"heading": "조사 개요", "content": f"주제: **{query}**"},
            {"heading": "웹 검색 결과", "content": document_writer.format_search_results(web_dicts)},
            {"heading": "관련 논문", "content": document_writer.format_paper_results(paper_dicts)},
        ]
        report = document_writer.write_report(f"선행조사: {query}", sections)

        result = {
            "query": query,
            "web_results": len(web_dicts),
            "paper_results": len(paper_dicts),
            "report_preview": report[:500],
        }

        # 4. 파일 저장
        if save:
            safe_name = query.replace(" ", "_")[:30]
            path = document_writer.save_to_project(f"research_{safe_name}.md", report)
            result["report_path"] = path

        logger.info("[research] 조사 완료: 웹 %d건, 논문 %d건", len(web_dicts), len(paper_dicts))
        return result
